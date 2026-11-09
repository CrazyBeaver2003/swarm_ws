import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from swarm_interfaces.msg import (
    FixedWingCommand,
    MeshEstimate,
    VehicleState,
)

from .follower_core import Follower, State


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')

        defaults = {
            'leader_id': 1,
            'follower_id': 2,
            'external_nav_state': 23,
            'frame_id': 'swarm_map',
            'origin_alt_amsl_m': 0.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.leader = self.get_parameter('leader_id').value
        self.follower = self.get_parameter('follower_id').value
        self.external = self.get_parameter('external_nav_state').value
        self.frame = self.get_parameter('frame_id').value
        self.origin = self.get_parameter('origin_alt_amsl_m').value

        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Run with use_sim_time:=true')

        if not (
            1 <= self.leader <= 255
            and 1 <= self.follower <= 255
            and self.leader != self.follower
            and 23 <= self.external <= 30
            and self.frame
            and math.isfinite(self.origin)
        ):
            raise ValueError('Invalid controller parameters')

        self.cache = {}
        self.subscriptions_ = []

        for vehicle_id in (self.leader, self.follower):
            for kind, message_type, topic in (
                ('mesh', MeshEstimate, 'mesh/estimate'),
                ('state', VehicleState, 'state'),
            ):
                key = (vehicle_id, kind)

                self.subscriptions_.append(
                    self.create_subscription(
                        message_type,
                        f'/plane_{vehicle_id}/{topic}',
                        lambda msg, key=key: self.cache.update({
                            key: (msg, time.monotonic())
                        }),
                        qos_profile_sensor_data,
                    )
                )

        self.commands = self.create_publisher(
            FixedWingCommand,
            f'/plane_{self.follower}/swarm_control/command',
            1,
        )

        self.targets = self.create_publisher(
            PointStamped,
            '~/target',
            1,
        )

        self.hold = self.create_client(
            Trigger,
            f'/plane_{self.follower}/flight/hold',
        )

        self.reset_service = self.create_service(
            Trigger,
            '~/reset',
            self.reset,
        )

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.core = Follower()
        self.active = False
        self.fault = False
        self.counters = None

        self.last_sim = None
        self.clock_at = time.monotonic()
        self.last_stamp = 0
        self.last_log = 0.0

        self.timer = self.create_timer(
            0.05,
            self.tick,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'Waiting for two airborne aircraft; no automatic activation'
        )

    def sample(self, vehicle_id, kind, sim, wall):
        entry = self.cache.get((vehicle_id, kind))

        if entry is None:
            raise ValueError(f'plane_{vehicle_id}: no {kind}')

        msg, received = entry

        stamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )
        age = sim - stamp

        if (
            msg.vehicle_id != vehicle_id
            or msg.header.frame_id != self.frame
            or not -0.02 <= age <= 0.3
            or wall - received > 1.0
        ):
            raise ValueError(
                f'plane_{vehicle_id}: stale/wrong {kind}'
            )

        return msg, max(age, 0.0)

    def inputs(self, sim, wall):
        states = []
        counters = []
        statuses = []

        for vehicle_id in (self.leader, self.follower):
            status, _ = self.sample(
                vehicle_id, 'state', sim, wall
            )
            mesh, age = self.sample(
                vehicle_id, 'mesh', sim, wall
            )

            received = self.cache[(vehicle_id, 'state')][1]
            status_age = status.status_age_s + wall - received

            if (
                not status.status_valid
                or not status.armed
                or status.failsafe
                or not math.isfinite(status.status_age_s)
                or status.status_age_s < 0.0
                or not 0.0 <= status_age <= 1.5
            ):
                raise ValueError(
                    f'plane_{vehicle_id}: not ready for formation'
                )

            values = (
                mesh.position.x,
                mesh.position.y,
                mesh.position.z,
                mesh.velocity.x,
                mesh.velocity.y,
                mesh.velocity.z,
            )

            if (
                not all(math.isfinite(v) for v in values)
                or mesh.status not in (
                    MeshEstimate.VALID,
                    MeshEstimate.DEGRADED,
                )
                or not 0.2 <= mesh.quality <= 1.0
                or mesh.used_station_count < 3
                or not 0.0 <= mesh.measurement_age_s <= 0.5 - age
            ):
                raise ValueError(
                    f'plane_{vehicle_id}: invalid mesh estimate'
                )

            x, y, z, vx, vy, vz = values

            # Небольшой прогноз от времени сообщения к текущему.
            state = State(
                x + vx * age,
                y + vy * age,
                z + vz * age,
                vx,
                vy,
            )

            if state.speed < 5.0:
                raise ValueError(
                    f'plane_{vehicle_id}: insufficient ground speed'
                )

            states.append(state)
            counters.append(mesh.reset_counter)
            statuses.append(status)

        return states, tuple(counters), statuses

    def stop(self, reason, request_hold=True):
        if self.fault:
            return

        self.fault = True

        self.get_logger().error(
            f'Latched fault: {reason}. '
            'Commands stopped; reset required'
        )

        # Не перезаписываем другой режим, выбранный оператором.
        try:
            status, _ = self.sample(
                self.follower,
                'state',
                self.get_clock().now().nanoseconds * 1e-9,
                time.monotonic(),
            )

            request_hold = (
                request_hold
                and status.status_valid
                and not status.failsafe
                and status.nav_state == self.external
                and status.nav_state_user_intention == self.external
            )

        except ValueError:
            request_hold = False

        if request_hold and self.hold.service_is_ready():
            future = self.hold.call_async(Trigger.Request())
            future.add_done_callback(self.hold_result)

    def hold_result(self, future):
        try:
            result = future.result()

            self.get_logger().warning(
                f'Hold request: {result.success}; {result.message}'
            )

        except Exception as error:
            self.get_logger().error(
                f'Hold request failed: {error}'
            )

    def reset(self, request, response):
        try:
            self.inputs(
                self.get_clock().now().nanoseconds * 1e-9,
                time.monotonic(),
            )

            status = self.cache[(self.follower, 'state')][0]

            if (
                status.nav_state != 4
                or status.nav_state_user_intention != 4
            ):
                raise ValueError(
                    'Follower must be in Hold, including user intention'
                )

            self.core = Follower()
            self.active = False
            self.fault = False
            self.counters = None
            self.last_sim = None
            self.clock_at = time.monotonic()

            response.success = True
            response.message = (
                'Reset; commands can resume. '
                'Activate Swarm Control separately'
            )

        except ValueError as error:
            response.success = False
            response.message = str(error)

        return response

    def tick(self):
        if self.fault:
            return

        wall = time.monotonic()
        sim = self.get_clock().now().nanoseconds * 1e-9

        if self.last_sim is None:
            self.last_sim = sim
            self.clock_at = wall
            return

        dt = sim - self.last_sim

        if dt == 0.0:
            if self.active and wall - self.clock_at > 1.0:
                self.stop('simulation clock stopped')
            return

        self.last_sim = sim
        self.clock_at = wall

        if dt < 0.0 or dt > 0.25:
            if self.active:
                self.stop('simulation time jump')
            else:
                self.core = Follower()

                if dt < 0.0:
                    self.cache.clear()
                    self.counters = None

            return

        try:
            states, counters, statuses = self.inputs(sim, wall)

        except ValueError as error:
            if self.active:
                self.stop(str(error))
            else:
                self.core = Follower()
                self.counters = None

                if wall - self.last_log > 3.0:
                    self.get_logger().info(f'Waiting: {error}')
                    self.last_log = wall

            return

        status = statuses[1]
        active = status.nav_state == self.external

        if self.active and not active:
            self.stop(
                'follower left Swarm Control',
                request_hold=False,
            )
            return

        if (
            active
            and self.counters is not None
            and counters != self.counters
        ):
            self.stop('localization reset')
            return

        if not active and counters != self.counters:
            self.core = Follower()

        if status.nav_state not in (4, self.external):
            self.core = Follower()
            return

        if active and not self.active:
            self.core.previous = None
            self.get_logger().info(
                'Follower is in Swarm Control'
            )

        self.active = active
        self.counters = counters

        command, target = self.core.step(
            *states,
            dt,
            active,
        )

        stamp = self.system_clock.now()

        if stamp.nanoseconds <= self.last_stamp:
            if self.active:
                self.stop('system clock moved backwards')
            return

        self.last_stamp = stamp.nanoseconds

        msg = FixedWingCommand()
        msg.stamp = stamp.to_msg()
        msg.course_rad = command.course
        msg.altitude_amsl_m = self.origin + command.altitude

        # Приближение для SITL без ветра на небольшой высоте.
        msg.equivalent_airspeed_m_s = command.speed

        self.commands.publish(msg)

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.frame

        point.point.x, point.point.y, point.point.z = target

        self.targets.publish(point)

        if wall - self.last_log > 2.0:
            error_xy = math.hypot(
                target[0] - states[1].x,
                target[1] - states[1].y,
            )
            error_z = target[2] - states[1].z

            self.get_logger().info(
                f'active={active}, '
                f'slot error: xy={error_xy:.1f}m, '
                f'z={error_z:.1f}m'
            )

            self.last_log = wall


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = FollowerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()