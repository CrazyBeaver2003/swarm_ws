"""Application-layer mesh transport emulator; never consumes simulator truth."""
import heapq
import json
import math
import random
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from swarm_interfaces.msg import MeshEstimate, VehicleState


class PacketQueue:
    def __init__(self, seed=7, capacity=4096):
        if not isinstance(seed, int) or not isinstance(capacity, int) or not 1 <= capacity <= 100000:
            raise ValueError('Invalid queue seed/capacity')
        self.seed, self.capacity = seed, capacity
        self.epoch = 0
        self.reset()

    def reset(self):
        self.random = random.Random(self.seed)
        self.queue, self.counters = [], {}
        self.sequence, self.last_time = 0, None

    def clock(self, now):
        if self.last_time is not None and now < self.last_time:
            self.reset()
            self.epoch += 1
        self.last_time = now

    def add(self, key, packet, now, latency, jitter, loss):
        self.clock(now)
        count = self.counters.setdefault(key, dict(
            generated=0, dropped=0, delivered=0, pending=0,
            delay_sum_s=0.0, delay_count=0))
        count['generated'] += 1
        if self.random.random() < loss or len(self.queue) >= self.capacity:
            count['dropped'] += 1
            return
        delay = max(0.0, self.random.gauss(latency, jitter))
        self.sequence += 1
        heapq.heappush(self.queue, (now + delay, self.sequence, now, key, packet))
        count['pending'] += 1

    def deliver(self, now):
        self.clock(now)
        result = []
        while self.queue and self.queue[0][0] <= now:
            _, _, generated, key, packet = heapq.heappop(self.queue)
            count = self.counters[key]
            count['pending'] -= 1
            count['delivered'] += 1
            count['delay_count'] += 1
            count['delay_sum_s'] += now - generated
            result.append((key, packet))
        return result


class MeshNetwork(Node):
    def __init__(self):
        super().__init__('mesh_network')
        for key, value in dict(vehicle_ids=[1, 2, 3], latency_s=0.02,
                               jitter_s=0.005, loss_probability=0.005,
                               seed=7, capacity=4096).items():
            self.declare_parameter(key, value)
        ids = self.get_parameter('vehicle_ids').value
        if not 3 <= len(ids) <= 5 or len(set(ids)) != len(ids):
            raise ValueError('Mesh network requires 3..5 distinct vehicles')
        self.queue = PacketQueue(self.get_parameter('seed').value,
                                 self.get_parameter('capacity').value)
        self.validate_values({name: self.get_parameter(name).value for name in
                              ('latency_s', 'jitter_s', 'loss_probability')})
        self.add_on_set_parameters_callback(self.validate)
        self.outputs, self.inputs = {}, []
        for i in ids:
            for kind, message_type, suffix in (
                ('state', VehicleState, 'state'), ('estimate', MeshEstimate, 'mesh/estimate')):
                key = (i, kind)
                self.outputs[key] = self.create_publisher(
                    message_type, f'/mesh/plane_{i}/{suffix}', qos_profile_sensor_data)
                self.inputs.append(self.create_subscription(
                    message_type, f'/plane_{i}/{suffix}',
                    lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data))
        self.stats = self.create_publisher(String, '/swarm/network/stats', 10)
        self.delivery_timer = self.create_timer(0.005, self.deliver)
        self.stats_timer = self.create_timer(1.0, self.publish_stats)

    @staticmethod
    def validate_values(values):
        for name, maximum in [('latency_s', 5.0), ('jitter_s', 2.0), ('loss_probability', 1.0)]:
            value = values[name]
            if not math.isfinite(value) or not 0.0 <= value <= maximum:
                raise ValueError(f'{name} must be finite in [0, {maximum}]')

    def validate(self, params):
        values = {n: self.get_parameter(n).value for n in ('latency_s', 'jitter_s', 'loss_probability')}
        values.update({p.name: p.value for p in params if p.name in values})
        try:
            self.validate_values(values)
            if any(p.name in ('vehicle_ids', 'seed', 'capacity') for p in params):
                raise ValueError('vehicle_ids, seed and capacity are startup-only')
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def receive(self, key, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.queue.add(key, (message, time.monotonic()), now,
                       self.get_parameter('latency_s').value,
                       self.get_parameter('jitter_s').value,
                       self.get_parameter('loss_probability').value)

    def deliver(self):
        for key, (message, received) in self.queue.deliver(self.get_clock().now().nanoseconds * 1e-9):
            if key[1] == 'state':
                # Source ages use wall time, unlike header/transport SIM timestamps.
                wall_delay = max(0.0, time.monotonic() - received)
                for field in ('position_age_s', 'velocity_age_s', 'attitude_age_s',
                              'altitude_age_s', 'status_age_s'):
                    setattr(message, field, getattr(message, field) + wall_delay)
            self.outputs[key].publish(message)

    def publish_stats(self):
        self.stats.publish(String(data=json.dumps(dict(
            stamp_s=self.get_clock().now().nanoseconds * 1e-9, epoch=self.queue.epoch,
            parameters={n: self.get_parameter(n).value for n in
                        ('latency_s', 'jitter_s', 'loss_probability')},
            links=[dict(vehicle_id=i, kind=kind, **values)
                   for (i, kind), values in sorted(self.queue.counters.items())]), allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = MeshNetwork()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
