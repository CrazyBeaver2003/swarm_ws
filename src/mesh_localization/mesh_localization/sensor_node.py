from dataclasses import asdict
import heapq
import json
import math

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from swarm_interfaces.msg import BaroHeight, RangeMeasurement

from .core import RttSimulator, SensorConfig
from .ros_parameters import (
    fixed_parameter, require_positive, seconds, set_stamp, station_parameters,
)

class RttSensorNode(Node):
    def __init__(self):
        super().__init__("rtt_sensor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.declare_parameter('drop_all_ranges', False)
        self.declare_parameter('drop_baro', False)
        self.declare_parameter('disabled_station_ids', [], ParameterDescriptor(dynamic_typing=True))

        self.frame_id = fixed_parameter(self, 'frame_id', "swarm_map")
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')

        self.stations = station_parameters(self)
        self.vehicle_ids = fixed_parameter(self, "vehicle_ids", [1,2,3])
        if (not self.vehicle_ids or len(set(self.vehicle_ids)) != len(self.vehicle_ids) or
                any(not 1 <= vehicle_id <= 255 for vehicle_id in self.vehicle_ids)):
            raise ValueError('vehicle_ids must be distinct PX4 system IDs in [1, 255]')

        rate = require_positive(fixed_parameter(self, 'rate_hz', 20.0), 'rate_hz')
        if rate > 200:
            raise ValueError('rate_hz must not exceed 200')
        self.max_pose_age = require_positive(
            fixed_parameter(self, 'max_pose_age_s', 0.2), 'max_pose_age_s')
        config_values = {}
        for name, default in asdict(SensorConfig()).items():
            if name in ('seed', 'turnaround_delay_ns', 'calibrated_turnaround_delay_ns'):
                config_values[name] = fixed_parameter(self, name, default)
            else:
                config_values[name] = self.declare_parameter(name, default).value
        self.simulator = RttSimulator(SensorConfig(**config_values))
        self.add_on_set_parameters_callback(self.validate_runtime_parameters)

        self.range_publishers = {}
        self.baro_publishers = {}
        self.pose_subscriptions = []
        self.poses = {}
        self.last_sample_stamp = {}
        self.queue = []
        self.queue_sequence = 0
        self.last_now = None
        self.epoch = 0

        for vehicle_id in self.vehicle_ids:
            prefix = f"/plane_{vehicle_id}/mesh"
            self.range_publishers[vehicle_id] = self.create_publisher(
                RangeMeasurement,
                f"{prefix}/ranges",
                qos_profile_sensor_data
            )
            self.baro_publishers[vehicle_id] = self.create_publisher(
                BaroHeight,
                f"{prefix}/baro",
                qos_profile_sensor_data,
            )
            self.pose_subscriptions.append(self.create_subscription(
                PoseStamped,
                f"/simulation/plane_{vehicle_id}/pose",
                lambda message, vehicle = vehicle_id: self.receive_pose(vehicle, message),
                qos_profile_sensor_data
            ))

        self.stats_publisher = self.create_publisher(String, '/simulation/ranging_stats', 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        # Separate delivery polling does not quantize a 40ms link latency to a
        # 50ms measurement period. Both timers follow /clock, including pauses.
        self.delivery_timer = self.create_timer(0.005, self.deliver_pending)
        self.stats_timer = self.create_timer(1.0, self.publish_stats)
        self.get_logger().info(
            f'RTT sensor: vehicles={self.vehicle_ids}, anchors={list(self.stations)}, '
            f'frame={self.frame_id}; ground truth is confined to the sensor boundary')

    def validate_runtime_parameters(self, parameters):
        updates = {item.name: item.value for item in parameters}
        mutable_config = {
            key: value for key, value in updates.items()
            if key in ('rtt_noise_std_ns', 'delivery_latency_s', 'delivery_jitter_s',
                        'loss_probability', 'baro_noise_std_m')}
        try:
            SensorConfig(**{**asdict(self.simulator.config), **mutable_config})
            disabled = updates.get('disabled_station_ids', [])
            if any(type(item) is not int or item not in self.stations for item in disabled):
                raise ValueError('disabled_station_ids must refer to configured station IDs')
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def receive_pose(self, vehicle_id, message):
        position = message.pose.position
        if (message.header.frame_id != self.frame_id or
                not all(math.isfinite(value) for value in (position.x, position.y, position.z))):
            self.get_logger().warning(
                'Ignoring a nonfinite pose or a pose outside the common ENU frame',
                throttle_duration_sec=5.0)
            return
        self.poses[vehicle_id] = message

    def check_clock(self, now):
        if self.last_now is not None and now < self.last_now - 1e-6:
            # A reset is a new deterministic run, not a delayed old packet.
            self.queue.clear()
            self.poses.clear()
            self.last_sample_stamp.clear()
            self.simulator = RttSimulator(self.simulator.config)
            self.epoch += 1
        self.last_now = now

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.check_clock(now)
        self.simulator.set_config(**{
            name: self.get_parameter(name).value for name in
            ('rtt_noise_std_ns', 'delivery_latency_s', 'delivery_jitter_s',
             'loss_probability', 'baro_noise_std_m')})
        disabled = set(self.get_parameter('disabled_station_ids').value)
        drop_all = self.get_parameter('drop_all_ranges').value
        drop_baro = self.get_parameter('drop_baro').value
        for vehicle_id, pose in list(self.poses.items()):
            stamp = seconds(pose.header.stamp)
            if (stamp == self.last_sample_stamp.get(vehicle_id) or
                    not 0 <= now - stamp <= self.max_pose_age):
                continue
            self.last_sample_stamp[vehicle_id] = stamp
            point = pose.pose.position
            position = (point.x, point.y, point.z)
            if not drop_baro:
                height = self.simulator.sample_height(vehicle_id, point.z, stamp)
                message = BaroHeight()
                message.header.frame_id = self.frame_id
                set_stamp(message.header.stamp, stamp)
                message.vehicle_id = vehicle_id
                message.height_m = height.height_m
                message.variance_m2 = height.variance_m2
                message.valid = height.valid
                self.baro_publishers[vehicle_id].publish(message)
            for station_id, station in self.stations.items():
                packet = self.simulator.sample_range(
                    vehicle_id, station_id, position, station, stamp,
                    disabled=drop_all or station_id in disabled)
                if packet is not None:
                    self.queue_sequence += 1
                    heapq.heappush(self.queue, (packet.delivery_stamp, self.queue_sequence, packet))
        self.deliver_pending()

    def deliver_pending(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.check_clock(now)
        while self.queue and self.queue[0][0] <= now:
            _, _, packet = heapq.heappop(self.queue)
            observation = packet.observation
            message = RangeMeasurement()
            message.header.frame_id = self.frame_id
            set_stamp(message.header.stamp, observation.stamp)
            message.vehicle_id = packet.vehicle_id
            message.station_id = observation.station_id
            message.sequence = observation.sequence
            message.rtt_ns = observation.rtt_ns
            message.turnaround_delay_ns = observation.turnaround_delay_ns
            message.range_m = observation.range_m
            message.variance_m2 = observation.variance_m2
            message.valid = observation.valid
            self.range_publishers[packet.vehicle_id].publish(message)
            self.simulator.mark_delivered(packet.vehicle_id, observation.station_id)

    def publish_stats(self):
        links = []
        for (vehicle_id, station_id), stats in sorted(self.simulator.counters.items()):
            links.append({
                'vehicle_id': vehicle_id, 'station_id': station_id, **stats,
                'pending': stats['generated'] - stats['dropped'] - stats['delivered']})
        message = String()
        message.data = json.dumps({
            'stamp_s': self.get_clock().now().nanoseconds * 1e-9,
            'epoch': self.epoch, 'frame_id': self.frame_id,
            'parameters': asdict(self.simulator.config),
            'drop_all_ranges': self.get_parameter('drop_all_ranges').value,
            'drop_baro': self.get_parameter('drop_baro').value,
            'disabled_station_ids': self.get_parameter('disabled_station_ids').value,
            'links': links}, sort_keys=True)
        self.stats_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RttSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
