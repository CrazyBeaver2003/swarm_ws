"""ROS adapter for RTT and barometric localization."""

from dataclasses import asdict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from swarm_interfaces.msg import (
    BaroHeight,
    MeshEstimate,
    RangeMeasurement,
)

from .core import (
    HeightObservation,
    LocalizerConfig,
    MeshLocalizer,
    RangeObservation,
)
from .ros_parameters import (
    fixed_parameter,
    require_positive,
    seconds,
    set_stamp,
    station_parameters,
)


class MeshLocalizerNode(Node):
    def __init__(self):
        super().__init__(
            'mesh_localizer',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
        )

        self.vehicle_id = fixed_parameter(self, 'vehicle_id', 1)
        if not 1 <= self.vehicle_id <= 255:
            raise ValueError(
                'vehicle_id must be a PX4 system ID in [1, 255]'
            )

        self.frame_id = fixed_parameter(
            self, 'frame_id', 'swarm_map'
        )
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')

        stations = station_parameters(self)

        config = LocalizerConfig(**{
            name: fixed_parameter(self, name, default)
            for name, default in asdict(LocalizerConfig()).items()
        })

        self.localizer = MeshLocalizer(stations, config)

        rate = require_positive(
            fixed_parameter(self, 'publish_rate_hz', 20.0),
            'publish_rate_hz',
        )
        if rate > 200:
            raise ValueError('publish_rate_hz must not exceed 200')

        prefix = f'/plane_{self.vehicle_id}/mesh'

        self.range_subscription = self.create_subscription(
            RangeMeasurement,
            f'{prefix}/ranges',
            self.receive_range,
            qos_profile_sensor_data,
        )

        self.baro_subscription = self.create_subscription(
            BaroHeight,
            f'{prefix}/baro',
            self.receive_height,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            MeshEstimate,
            f'{prefix}/estimate',
            10,
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self.publish_estimate,
        )

        self.get_logger().info(
            f'Plane {self.vehicle_id}: RTT/baro estimator '
            f'in {self.frame_id}, anchors={list(stations)}'
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive_range(self, message):
        if (
            message.vehicle_id != self.vehicle_id
            or message.header.frame_id != self.frame_id
        ):
            return

        observation = RangeObservation(
            stamp=seconds(message.header.stamp),
            station_id=message.station_id,
            sequence=message.sequence,
            rtt_ns=message.rtt_ns,
            turnaround_delay_ns=message.turnaround_delay_ns,
            range_m=message.range_m,
            variance_m2=message.variance_m2,
            valid=message.valid,
        )

        self.localizer.add_range(
            observation,
            self.now_seconds(),
        )

    def receive_height(self, message):
        if (
            message.vehicle_id != self.vehicle_id
            or message.header.frame_id != self.frame_id
        ):
            return

        observation = HeightObservation(
            stamp=seconds(message.header.stamp),
            height_m=message.height_m,
            variance_m2=message.variance_m2,
            valid=message.valid,
        )

        self.localizer.add_height(
            observation,
            self.now_seconds(),
        )

    def publish_estimate(self):
        estimate = self.localizer.estimate(self.now_seconds())

        message = MeshEstimate()
        message.header.frame_id = self.frame_id
        set_stamp(message.header.stamp, estimate.stamp)

        message.vehicle_id = self.vehicle_id

        (
            message.position.x,
            message.position.y,
            message.position.z,
        ) = map(float, estimate.position)

        (
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
        ) = map(float, estimate.velocity)

        message.position_covariance = (
            estimate.position_covariance.ravel().tolist()
        )
        message.velocity_covariance = (
            estimate.velocity_covariance.ravel().tolist()
        )

        message.status = estimate.status
        message.quality = estimate.quality
        message.used_station_count = estimate.used_station_count
        message.measurement_age_s = estimate.measurement_age_s
        message.reset_counter = estimate.reset_counter

        self.publisher.publish(message)

        if self.localizer.last_rejection:
            self.get_logger().warning(
                'Last rejected measurement: '
                f'{self.localizer.last_rejection}',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MeshLocalizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()