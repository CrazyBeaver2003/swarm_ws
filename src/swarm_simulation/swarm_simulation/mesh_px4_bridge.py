"""Forward fresh mesh estimates to PX4 after checking the shared origin."""

import math

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from swarm_interfaces.msg import MeshEstimate


def px4_topic(namespace, direction, name, message_type):
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    version = message_type.MESSAGE_VERSION
    suffix = f'_v{version}' if version else ''
    return f'{prefix}/fmu/{direction}/{name}{suffix}'


def estimate_to_odometry(message):
    if message.status not in (
        MeshEstimate.VALID,
        MeshEstimate.DEGRADED,
    ):
        return None

    if (
        not math.isfinite(message.quality)
        or not 0 < message.quality <= 1
    ):
        return None

    p, v = message.position, message.velocity

    values = [p.x, p.y, p.z, v.x, v.y, v.z]
    covariance = (
        list(message.position_covariance)
        + list(message.velocity_covariance)
    )

    if not all(
        math.isfinite(x) and abs(x) <= 3.402823466e38
        for x in values + covariance
    ):
        return None

    # Диагонали covariance после преобразования ENU → NED.
    order = [4, 0, 8]
    position_variance = [
        message.position_covariance[i] for i in order
    ]
    velocity_variance = [
        message.velocity_covariance[i] for i in order
    ]

    if any(
        x < 0
        for x in position_variance + velocity_variance
    ):
        return None

    output = VehicleOdometry()

    # В закреплённой версии PX4 DDS заменяет нули
    # временем приёма сообщения внутри PX4.
    output.timestamp = 0
    output.timestamp_sample = 0

    output.pose_frame = VehicleOdometry.POSE_FRAME_NED
    output.position = [p.y, p.x, -p.z]

    # Локализатор оценивает положение и скорость.
    # Ориентацию продолжает определять PX4.
    output.q = [float('nan')] * 4

    output.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    output.velocity = [v.y, v.x, -v.z]
    output.angular_velocity = [float('nan')] * 3

    output.position_variance = [
        max(0.01, float(x)) for x in position_variance
    ]
    output.velocity_variance = [
        max(0.04, float(x)) for x in velocity_variance
    ]
    output.orientation_variance = [float('nan')] * 3

    output.reset_counter = message.reset_counter % 256
    output.quality = max(
        1, min(100, round(message.quality * 100))
    )

    return output


class MeshPx4Bridge(Node):
    def __init__(self):
        super().__init__(
            'mesh_px4_bridge',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
        )

        readonly = ParameterDescriptor(read_only=True)

        def fixed(name, default):
            return self.declare_parameter(
                name, default, readonly
            ).value

        self.vehicle_id = fixed('vehicle_id', 1)
        self.frame = fixed('frame_id', 'swarm_map')
        namespace = fixed('px4_namespace', '')

        self.lat = fixed(
            'origin_lat_deg', 47.397971057728974
        )
        self.lon = fixed(
            'origin_lon_deg', 8.546163739800146
        )
        self.alt = fixed('origin_alt_amsl_m', 0.0)

        self.header_timeout = fixed(
            'max_estimate_age_s', 0.2
        )
        self.measurement_timeout = fixed(
            'max_measurement_age_s', 0.25
        )
        self.reference_timeout = fixed(
            'reference_timeout_s', 1.0
        )

        if not 1 <= self.vehicle_id <= 255 or not self.frame:
            raise ValueError('invalid vehicle_id or frame_id')

        if (
            not math.isfinite(self.lat)
            or abs(self.lat) > 90
            or not math.isfinite(self.lon)
            or abs(self.lon) > 180
            or not math.isfinite(self.alt)
        ):
            raise ValueError('invalid shared origin')

        if not all(
            math.isfinite(x) and x > 0
            for x in (
                self.header_timeout,
                self.measurement_timeout,
                self.reference_timeout,
            )
        ):
            raise ValueError(
                'timeouts must be finite and positive'
            )

        self._position = None
        self._position_at = None
        self._last_now = None
        self._last_header = None
        self._last_source = None
        self._source_reset = None
        self._output_reset = 0
        self._need_reset = False

        target = px4_topic(
            namespace,
            'in',
            'vehicle_visual_odometry',
            VehicleOdometry,
        )
        local_topic = px4_topic(
            namespace,
            'out',
            'vehicle_local_position',
            VehicleLocalPosition,
        )
        mesh_topic = (
            f'/plane_{self.vehicle_id}/mesh/estimate'
        )

        self.publisher = self.create_publisher(
            VehicleOdometry, target, 10
        )
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            local_topic,
            self.on_position,
            qos_profile_sensor_data,
        )
        self.estimate_subscription = self.create_subscription(
            MeshEstimate,
            mesh_topic,
            self.on_estimate,
            10,
        )

        self.get_logger().info(
            f'{mesh_topic} -> {target}; '
            f'reference from {local_topic}'
        )

    def _check_clock(self, now):
        if self._last_now is not None and now < self._last_now:
            self._position = None
            self._position_at = None
            self._last_header = None
            self._last_source = None
            self._source_reset = None
            self._need_reset = True

        self._last_now = now

    def on_position(self, message):
        now = self.get_clock().now().nanoseconds
        self._check_clock(now)

        self._position = message
        self._position_at = now

    def _origin_ready(self, now):
        p = self._position

        return (
            p is not None
            and 0 <= now - self._position_at
            <= self.reference_timeout * 1e9
            and p.xy_global
            and p.z_global
            and all(
                math.isfinite(x)
                for x in (
                    p.ref_lat,
                    p.ref_lon,
                    p.ref_alt,
                )
            )
            and abs(p.ref_lat - self.lat) < 1e-7
            and abs(p.ref_lon - self.lon) < 1e-7
            and abs(p.ref_alt - self.alt) < 0.1
        )

    def _reject(self, reason):
        self.get_logger().warning(
            reason, throttle_duration_sec=2.0
        )

    def on_estimate(self, message):
        now = self.get_clock().now().nanoseconds
        self._check_clock(now)

        if (
            message.vehicle_id != self.vehicle_id
            or message.header.frame_id != self.frame
        ):
            return

        header = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        age = (now - header) / 1e9

        if not 0 <= age <= self.header_timeout:
            self._reject(
                'Mesh prediction timestamp is stale '
                'or in the future'
            )
            return

        if (
            not math.isfinite(message.measurement_age_s)
            or not 0 <= message.measurement_age_s
            <= self.measurement_timeout
        ):
            self._reject(
                'Underlying mesh measurements are stale'
            )
            return

        output = estimate_to_odometry(message)
        if output is None:
            return

        if not self._origin_ready(now):
            self._need_reset = True
            self._reject(
                'Waiting for a fresh PX4 reference '
                'matching config_reference.yaml'
            )
            return

        if (
            self._last_header is not None
            and header <= self._last_header
        ):
            return

        # Восстанавливаем время последнего исходного замера.
        source = header - round(
            message.measurement_age_s * 1e9
        )
        if source < 0:
            return

        changed = (
            self._source_reset is not None
            and message.reset_counter != self._source_reset
        )
        restart = changed or self._need_reset

        # Повторные прогнозы одного замера не считаем
        # независимыми новыми измерениями.
        if (
            not restart
            and self._last_source is not None
            and source <= self._last_source + 1000
        ):
            return

        if restart:
            self._output_reset = (
                self._output_reset + 1
            ) % 256

        output.reset_counter = self._output_reset
        self.publisher.publish(output)

        self._last_header = header
        self._last_source = source
        self._source_reset = message.reset_counter
        self._need_reset = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MeshPx4Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()