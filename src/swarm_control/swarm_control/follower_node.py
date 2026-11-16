import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import (
    FixedWingCommand,
    MeshEstimate,
    VehicleState,
)
from swarm_interfaces.srv import SetFormation

from .follower_core import Follower, State


class StatusNotFresh(ValueError):
    pass


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower')

        defaults = {
            'leader_id': 1,
            'follower_id': 2,
            'external_nav_state': 23,
            'frame_id': 'swarm_map',
            'origin_alt_amsl_m': 0.0,
            'behind_m': 10.0,
            'right_m': 0.0,
            'above_m': 0.0,
            'leader_peer_prefix': '',
            'slot_index': 1,
            'vehicle_count': 3,
            'entry_lane_spacing_m': 40.0,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.leader = self.get_parameter('leader_id').value
        self.follower = self.get_parameter('follower_id').value
        self.external = self.get_parameter('external_nav_state').value
        self.frame = self.get_parameter('frame_id').value
        self.origin = self.get_parameter('origin_alt_amsl_m').value
        self.behind = self.get_parameter('behind_m').value
        self.right = self.get_parameter('right_m').value
        self.above = self.get_parameter('above_m').value

        self.slot_index = self.get_parameter('slot_index').value
        self.vehicle_count = self.get_parameter('vehicle_count').value
        lane_spacing = self.get_parameter('entry_lane_spacing_m').value

        if not 2 <= self.vehicle_count <= 5 or not 1 <= self.slot_index < self.vehicle_count:
            raise ValueError('Need vehicle_count 2..5 and slot_index 1..vehicle_count-1')
        if not math.isfinite(lane_spacing) or not 20.0 <= lane_spacing <= 120.0:
            raise ValueError('entry_lane_spacing_m must be 20..120 m')
        row = (self.slot_index + 1) // 2
        side = -1.0 if self.slot_index % 2 else 1.0
        self.entry_right = side * row * lane_spacing

        if not (
            all(
                math.isfinite(value)
                for value in (self.behind, self.right, self.above)
            )
            and self.behind > 0.0
            and self.above >= 0.0
        ):
            raise ValueError(
                'Offsets must be finite; '
                'behind_m must be positive and above_m nonnegative'
            )

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

        peer_prefix = self.get_parameter('leader_peer_prefix').value.rstrip('/')
        if peer_prefix and not peer_prefix.startswith('/'):
            raise ValueError('leader_peer_prefix must be absolute or empty')
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
                        f'{peer_prefix if vehicle_id == self.leader else ""}/plane_{vehicle_id}/{topic}',
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

        self.formation_service = self.create_service(
            SetFormation,
            '~/set_formation',
            self.set_formation,
        )

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.core = self.make_controller()
        self.active = False
        self.fault = False
        self.counters = None

        self.last_sim = None
        self.clock_at = time.monotonic()
        self.last_stamp = 0
        self.last_log = 0.0
        self.status_wait_since = None
        self.last_entry_phase = None
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', status_qos)
        self.status_timer = self.create_timer(
            0.2, self.publish_status, clock=self.steady_clock,
        )

        self.timer = self.create_timer(
            0.05,
            self.tick,
            clock=self.steady_clock,
        )

        self.get_logger().info(
            'Waiting for two airborne aircraft; no automatic activation'
        )

    @staticmethod
    def slot_offset(index, formation, spacing):
        if formation == 'line':
            return index * spacing, 0.0
        row = (index + 1) // 2
        side = -1.0 if index % 2 else 1.0
        return row * spacing, side * row * spacing * 0.8

    def set_formation(self, request, response):
        try:
            formation = request.formation
            spacing = request.spacing_m

            if formation not in ('line', 'wedge'):
                raise ValueError('formation must be line or wedge')

            if (
                not math.isfinite(spacing)
                or not 10.0 <= spacing <= 120.0
            ):
                raise ValueError('spacing_m must be 10..120')

            if self.fault or not self.active:
                raise ValueError(
                    'Follower must be active and free of faults'
                )

            _, counters, statuses = self.inputs(
                self.get_clock().now().nanoseconds * 1e-9,
                time.monotonic(),
            )

            status = statuses[1]

            if (
                status.nav_state != self.external
                or status.nav_state_user_intention != self.external
                or counters != self.counters
            ):
                raise ValueError(
                    'Mode or localization changed; wait for status'
                )

            index = self.slot_index
            behind, right = self.slot_offset(index, formation, spacing)
            same_target = (abs(behind - self.behind) < 1e-9
                           and abs(right - self.right) < 1e-9)
            if not same_target and self.core.entry_phase in ('approach', 'merge'):
                raise ValueError('Formation entry in progress; wait for all followers to join')
            if not same_target and self.core.transition_remaining_s > 1e-6:
                raise ValueError('Slot transition in progress; wait before changing formation')

            # Every node computes the duration of the longest slot movement.
            # Equal progress prevents faster slots from cutting through their
            # neighbours when all aircraft fly at the same altitude.
            old_formation = 'line' if abs(self.right) < 1e-9 else 'wedge'
            old_spacing = self.behind / (index if old_formation == 'line' else (index + 1) // 2)
            longest = max(
                math.dist(self.slot_offset(i, old_formation, old_spacing),
                          self.slot_offset(i, formation, spacing))
                for i in range(1, self.vehicle_count)
            )
            transition_duration = longest / 1.0  # At most 1 m/s for the farthest slot.
            self.core.set_slot(behind, right, transition_duration_s=transition_duration)

            # Сохраняем принятую цель для последующих сбросов ядра.
            self.behind = behind
            self.right = right

            response.success = True
            response.message = (
                f'Accepted {formation}: '
                f'behind={behind:.1f}m, right={right:.1f}m. '
                'Transition is not yet complete'
            )

            self.get_logger().info(response.message)

        except ValueError as error:
            response.success = False
            response.message = str(error)

        return response
    
    def make_controller(self):
        old = getattr(self, 'core', None)

        controller = Follower(
            behind=old.behind if old is not None else self.behind,
            above=self.above,
            right=old.right if old is not None else self.right,
            entry_right_m=self.entry_right,
        )

        remaining = old.transition_remaining_s if old is not None else None
        controller.set_slot(self.behind, self.right, transition_duration_s=remaining)

        return controller

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'vehicle_id': self.follower,
            'frame_id': self.frame,
            'stamp_s': self.get_clock().now().nanoseconds * 1e-9,
            'active': self.active,
            'fault': self.fault,
            'entry_phase': self.core.entry_phase,
            'transition_remaining_s': self.core.transition_remaining_s,
            'entry_diagnostics': self.core.entry_diagnostics,
        }, allow_nan=False)))
    
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

    def check_statuses(self, sim, wall):
        statuses = []
        stale = []
        for vehicle_id in (self.leader, self.follower):
            status, _ = self.sample(vehicle_id, 'state', sim, wall)
            received = self.cache[(vehicle_id, 'state')][1]
            age = status.status_age_s + wall - received
            if not math.isfinite(status.status_age_s) or status.status_age_s < 0.0:
                raise ValueError(f'plane_{vehicle_id}: invalid status age')
            if not status.status_valid or age > 1.5:
                stale.append(
                    f'plane_{vehicle_id}: status stale; '
                    f'valid={status.status_valid}, '
                    f'source_age={status.status_age_s:.3f}s, total_age={age:.3f}s'
                )
            elif not status.armed or status.failsafe:
                raise ValueError(
                    f'plane_{vehicle_id}: armed={status.armed}, '
                    f'failsafe={status.failsafe}'
                )
            statuses.append(status)
        if stale:
            raise StatusNotFresh('; '.join(stale))
        return statuses

    def inputs(self, sim, wall):
        states = []
        counters = []
        statuses = self.check_statuses(sim, wall)

        for vehicle_id in (self.leader, self.follower):
            mesh, age = self.sample(vehicle_id, 'mesh', sim, wall)

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

            self.core = self.make_controller()
            self.active = False
            self.fault = False
            self.status_wait_since = None
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
                self.core = self.make_controller()

                if dt < 0.0:
                    self.cache.clear()
                    self.counters = None

            return

        try:
            states, counters, statuses = self.inputs(sim, wall)

        except StatusNotFresh as error:
            if self.active:
                if self.status_wait_since is None:
                    self.status_wait_since = wall
                    self.get_logger().warning(
                        f'{error}; pausing commands for up to 0.5s'
                    )
                elif wall - self.status_wait_since >= 0.5:
                    self.stop(f'Status did not recover: {error}')
            elif wall - self.last_log > 3.0:
                self.get_logger().info(f'Waiting: {error}')
                self.last_log = wall
            return

        except ValueError as error:
            self.status_wait_since = None
            if self.active:
                self.stop(str(error))
            else:
                self.core = self.make_controller()
                self.counters = None

                if wall - self.last_log > 3.0:
                    self.get_logger().info(f'Waiting: {error}')
                    self.last_log = wall

            return

        self.status_wait_since = None
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
            self.core = self.make_controller()

        if status.nav_state not in (4, self.external):
            self.core = self.make_controller()
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
        if self.core.entry_phase != self.last_entry_phase:
            self.get_logger().info(f'Formation entry phase: {self.core.entry_phase}')
            self.last_entry_phase = self.core.entry_phase

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
            entry = self.core.entry_diagnostics
            filtered = entry.get('filtered', {})
            entry_details = ''
            if filtered:
                entry_details = (
                    f", entry along/lateral={filtered['along_m']:.1f}/"
                    f"{filtered['lateral_m']:.1f}m, "
                    f"speed error={filtered['speed_m_s']:.1f}m/s, "
                    f"ready={entry['ready_s']:.1f}s, aligned={entry['aligned']}"
                )

            self.get_logger().info(
                f'active={active}, entry={self.core.entry_phase}, '
                f'slot error: xy={error_xy:.1f}m, '
                f'z={error_z:.1f}m{entry_details}'
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
