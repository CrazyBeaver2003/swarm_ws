"""Detection of current/predicted close approaches; not a collision-free planner."""
import math
import time
from itertools import combinations

from rclpy.qos import qos_profile_sensor_data
from swarm_interfaces.msg import MeshEstimate


def closest_approach(position_delta, velocity_delta, horizon):
    vv = sum(v * v for v in velocity_delta)
    t = 0.0 if vv < 1e-9 else max(0.0, min(horizon,
        -sum(p * v for p, v in zip(position_delta, velocity_delta)) / vv))
    return math.sqrt(sum((p + v * t) ** 2 for p, v in zip(position_delta, velocity_delta))), t


class SeparationGuard:
    def __init__(self, manager):
        self.m = manager
        self.distance = manager.declare_parameter('minimum_separation_m', 20.0).value
        self.horizon = manager.declare_parameter('separation_horizon_s', 8.0).value
        if not (math.isfinite(self.distance) and 5.0 <= self.distance <= 30.0
                and math.isfinite(self.horizon) and 0.0 <= self.horizon <= 15.0):
            raise ValueError('Invalid separation settings')
        self.mesh = {}
        self.subscriptions = [manager.create_subscription(
            MeshEstimate, f'/plane_{i}/mesh/estimate',
            lambda msg, i=i: self.mesh.update({i: (msg, time.monotonic())}),
            qos_profile_sensor_data, callback_group=manager.group) for i in manager.ids]

    def problem(self):
        now = time.monotonic()
        sim = self.m.get_clock().now().nanoseconds * 1e-9
        samples = {}
        for i, (msg, received) in self.mesh.items():
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = sim - stamp
            values = (msg.position.x, msg.position.y, msg.position.z,
                      msg.velocity.x, msg.velocity.y, msg.velocity.z)
            if (msg.vehicle_id == i and msg.header.frame_id == self.m.frame
                    and -0.02 <= age <= 0.3 and now - received <= 0.5
                    and msg.status in (MeshEstimate.VALID, MeshEstimate.DEGRADED)
                    and math.isfinite(msg.quality) and msg.quality >= 0.2
                    and all(math.isfinite(v) for v in values)):
                samples[i] = (tuple(p + max(0.0, age) * v for p, v in
                                   zip(values[:3], values[3:])), values[3:])
        for i, j in combinations(sorted(samples), 2):
            a, b = samples[i], samples[j]
            delta = tuple(x - y for x, y in zip(a[0], b[0]))
            velocity = tuple(x - y for x, y in zip(a[1], b[1]))
            distance, after = closest_approach(delta, velocity, self.horizon)
            if distance < self.distance:
                return f'Unsafe predicted separation plane_{i}/plane_{j}: {distance:.1f} m in {after:.1f} s'
        return ''
