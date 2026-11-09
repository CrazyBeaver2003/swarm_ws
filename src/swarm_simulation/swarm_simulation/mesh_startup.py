"""Initialize the shared PX4 origin and check mesh navigation."""

import math
import time

import rclpy
from px4_msgs.msg import (
    EstimatorStatusFlags,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as RosClock
from swarm_interfaces.msg import MeshEstimate


class MeshStartup(Node):
    def __init__(self):
        super().__init__('mesh_startup')
        readonly = ParameterDescriptor(read_only=True)

        def fixed(name, default):
            return self.declare_parameter(
                name, default, readonly
            ).value

        self.vehicle_id = fixed('vehicle_id', 1)
        self.frame = fixed('frame_id', 'swarm_map')

        namespace = fixed('px4_namespace', '').strip('/')
        self.prefix = '/' + namespace if namespace else ''

        self.lat = fixed(
            'origin_lat_deg', 47.397971057728974
        )
        self.lon = fixed(
            'origin_lon_deg', 8.546163739800146
        )
        self.alt = fixed('origin_alt_amsl_m', 0.0)
        self.timeout = fixed('startup_timeout_s', 90.0)

        if (
            not 1 <= self.vehicle_id <= 255
            or not self.frame
            or not all(
                math.isfinite(x)
                for x in (
                    self.lat, self.lon,
                    self.alt, self.timeout,
                )
            )
            or abs(self.lat) > 90
            or abs(self.lon) > 180
            or self.timeout <= 0
        ):
            raise ValueError('Invalid startup parameters')

        self.result = None
        self.cache = {}
        self.started = time.monotonic()
        self.sent_at = -math.inf
        self.attempts = 0
        self.origin_confirmed = False
        self.stable_since = None
        self.reason = 'Waiting for PX4'

        self.publisher = self.create_publisher(
            VehicleCommand,
            self.topic(
                'in', 'vehicle_command', VehicleCommand
            ),
            10,
        )

        self.inputs = []

        for key, name, kind in (
            ('status', 'vehicle_status', VehicleStatus),
            (
                'landed',
                'vehicle_land_detected',
                VehicleLandDetected,
            ),
            (
                'local',
                'vehicle_local_position',
                VehicleLocalPosition,
            ),
            (
                'flags',
                'estimator_status_flags',
                EstimatorStatusFlags,
            ),
        ):
            self.inputs.append(
                self.create_subscription(
                    kind,
                    self.topic('out', name, kind),
                    lambda msg, key=key: self.receive(
                        key, msg
                    ),
                    qos_profile_sensor_data,
                )
            )

        self.inputs.append(
            self.create_subscription(
                MeshEstimate,
                f'/plane_{self.vehicle_id}/mesh/estimate',
                lambda msg: self.receive('mesh', msg),
                10,
            )
        )

        self.inputs.append(
            self.create_subscription(
                RosClock,
                '/clock',
                lambda msg: self.receive('clock', msg),
                10,
            )
        )

        self.timer = self.create_timer(
            0.1,
            self.step,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def topic(self, direction, name, kind):
        version = getattr(kind, 'MESSAGE_VERSION', 0)
        suffix = f'_v{version}' if version else ''

        return (
            f'{self.prefix}/fmu/'
            f'{direction}/{name}{suffix}'
        )

    def receive(self, key, message):
        self.cache[key] = (
            message,
            time.monotonic(),
        )

    def fresh(self, key, max_age):
        item = self.cache.get(key)

        if item is None:
            return None

        message, received_at = item
        age = time.monotonic() - received_at

        return message if 0 <= age <= max_age else None

    def origin_matches(self, position):
        return (
            position is not None
            and position.xy_global
            and position.z_global
            and all(
                math.isfinite(x)
                for x in (
                    position.ref_lat,
                    position.ref_lon,
                    position.ref_alt,
                )
            )
            and abs(position.ref_lat - self.lat) < 1e-7
            and abs(position.ref_lon - self.lon) < 1e-7
            and abs(position.ref_alt - self.alt) < 0.1
        )

    def wait(self, reason):
        self.reason = reason
        self.stable_since = None

        self.get_logger().info(
            reason,
            throttle_duration_sec=5.0,
        )

    def finish(self, result, message):
        self.result = result
        self.timer.cancel()

        if result == 0:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def step(self):
        if self.result is not None:
            return

        now = time.monotonic()

        if now - self.started > self.timeout:
            self.finish(
                2,
                f'Startup timeout: {self.reason}',
            )
            return

        status = self.fresh('status', 2.5)
        landed = self.fresh('landed', 2.5)
        position = self.fresh('local', 0.5)

        if status is None or landed is None:
            self.wait(
                'Waiting for fresh vehicle status '
                'and land detector'
            )
            return

        if status.system_id != self.vehicle_id:
            self.finish(
                2,
                'Wrong PX4 system_id; '
                'check px4_namespace',
            )
            return

        if (
            status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            self.finish(
                2,
                'Startup initialization requires '
                'a disarmed aircraft',
            )
            return

        if (
            status.arming_state
            != VehicleStatus.ARMING_STATE_DISARMED
            or not landed.landed
        ):
            self.wait(
                'Waiting for disarmed aircraft '
                'on the ground'
            )
            return

        if not self.origin_confirmed:
            if self.origin_matches(position):
                self.origin_confirmed = True
                self.get_logger().info(
                    'Shared EKF origin confirmed'
                )

            elif now - self.sent_at >= 2.0:
                if self.attempts >= 3:
                    self.finish(
                        2,
                        'PX4 did not confirm '
                        'the shared origin',
                    )
                    return

                if (
                    self.publisher.get_subscription_count()
                    == 0
                ):
                    self.wait(
                        'Waiting for the PX4 '
                        'command subscriber'
                    )
                    return

                command = VehicleCommand()
                command.command = (
                    VehicleCommand
                    .VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
                )
                command.param5 = self.lat
                command.param6 = self.lon
                command.param7 = self.alt
                command.target_system = self.vehicle_id
                command.target_component = (
                    status.component_id
                )
                command.source_system = 255
                command.source_component = 191
                command.from_external = True

                self.publisher.publish(command)

                self.sent_at = now
                self.attempts += 1

                self.wait(
                    'Waiting for origin confirmation, '
                    f'attempt {self.attempts}/3'
                )
                return

            else:
                return

        if not self.origin_matches(position):
            self.wait(
                'Waiting for fresh matching origin; '
                'automatic reset is disabled'
            )
            return

        if (
            not all((
                position.xy_valid,
                position.z_valid,
                position.v_xy_valid,
                position.v_z_valid,
            ))
            or position.dead_reckoning
            or not all(
                math.isfinite(x)
                for x in (
                    position.x,
                    position.y,
                    position.z,
                    position.vx,
                    position.vy,
                    position.vz,
                )
            )
        ):
            self.wait(
                'Waiting for valid PX4 '
                'position and velocity'
            )
            return

        flags = self.fresh('flags', 2.5)

        if (
            flags is None
            or not all((
                flags.cs_tilt_align,
                flags.cs_yaw_align,
                flags.cs_ev_pos,
                flags.cs_ev_hgt,
                flags.cs_ev_vel,
            ))
            or flags.cs_inertial_dead_reckoning
            or flags.cs_wind_dead_reckoning
        ):
            self.wait(
                'Waiting for active mesh '
                'correction in EKF'
            )
            return

        if any((
            flags.cs_gnss_pos,
            flags.cs_gps_hgt,
            flags.cs_gnss_vel,
            flags.cs_gnss_yaw,
        )):
            self.wait(
                'GNSS correction is active; '
                'expected EKF2_GPS_CTRL=0'
            )
            return

        mesh = self.fresh('mesh', 0.5)
        clock = self.fresh('clock', 0.5)

        if (
            mesh is None
            or mesh.vehicle_id != self.vehicle_id
            or mesh.header.frame_id != self.frame
            or mesh.status != MeshEstimate.VALID
            or mesh.used_station_count < 3
            or not math.isfinite(
                mesh.measurement_age_s
            )
            or not 0 <= mesh.measurement_age_s <= 0.25
            or not math.isfinite(mesh.quality)
            or not 0 < mesh.quality <= 1
        ):
            self.wait(
                'Waiting for a fresh valid mesh estimate'
            )
            return

        if clock is None:
            self.wait(
                'Waiting for the simulation clock'
            )
            return

        sim_now = (
            clock.clock.sec
            + clock.clock.nanosec * 1e-9
        )
        stamp = (
            mesh.header.stamp.sec
            + mesh.header.stamp.nanosec * 1e-9
        )

        if (
            sim_now <= 0
            or stamp <= 0
            or not -0.02 <= sim_now - stamp <= 0.2
            or not all(
                math.isfinite(x)
                for x in (
                    mesh.position.x,
                    mesh.position.y,
                    mesh.position.z,
                    mesh.velocity.x,
                    mesh.velocity.y,
                    mesh.velocity.z,
                )
            )
        ):
            self.wait(
                'Mesh prediction has an invalid '
                'timestamp or state'
            )
            return

        if self.stable_since is None:
            self.stable_since = now

        if now - self.stable_since >= 2.0:
            self.finish(
                0,
                'Mesh navigation READY; '
                'startup check completed',
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 2

    try:
        node = MeshStartup()

        while rclpy.ok() and node.result is None:
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        result = (
            node.result
            if node.result is not None
            else 2
        )

    except KeyboardInterrupt:
        result = 130

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.try_shutdown()

    raise SystemExit(result)