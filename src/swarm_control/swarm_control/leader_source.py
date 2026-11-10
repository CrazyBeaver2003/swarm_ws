import math
import time

from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from swarm_interfaces.msg import FixedWingCommand, MeshEstimate
from swarm_interfaces.srv import SetLeaderCommand


class StatusNotFresh(ValueError):
    pass


class LeaderSource:
    """A bounded command stream for one leader; mode changes belong to the manager."""

    def __init__(self, manager, external):
        self.m = manager
        self.vehicle_id = manager.ids[0]
        self.external = external
        if not 23 <= external <= 30:
            raise ValueError('external_nav_state must be 23..30')
        if not manager.has_parameter('origin_alt_amsl_m'):
            manager.declare_parameter('origin_alt_amsl_m', 0.0)
        self.origin = float(manager.get_parameter('origin_alt_amsl_m').value)
        if not math.isfinite(self.origin):
            raise ValueError('Invalid altitude origin')

        self.mesh = None
        self.running = False
        self.fault = ''
        self.seen_external = False
        self.published_at = 0.0
        self.last_stamp = 0
        self.last_sim = None
        self.clock_at = time.monotonic()
        self.status_wait_since = None
        self.counter = None
        self.course = self.altitude = 0.0
        self.speed = 15.0
        self.target = None
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self.subscription = manager.create_subscription(
            MeshEstimate, f'/plane_{self.vehicle_id}/mesh/estimate',
            self.on_mesh, qos_profile_sensor_data,
            callback_group=manager.group,
        )
        self.publisher = manager.create_publisher(
            FixedWingCommand,
            f'/plane_{self.vehicle_id}/swarm_control/command', 1,
        )
        self.service = manager.create_service(
            SetLeaderCommand, '/swarm/leader/set_command', self.set_command,
            callback_group=manager.group,
        )
        self.timer = manager.create_timer(
            0.05, self.tick, clock=manager.steady,
            callback_group=manager.group,
        )

    def on_mesh(self, message):
        self.mesh = (message, time.monotonic())

    def sample(self, entry, sim, wall):
        if entry is None:
            raise ValueError('Leader telemetry unavailable')
        message, received = entry
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = sim - stamp
        if not (
            message.vehicle_id == self.vehicle_id
            and message.header.frame_id == self.m.frame
            and -0.02 <= age <= 0.3
            and 0.0 <= wall - received <= 1.0
        ):
            raise ValueError('Leader telemetry stale or wrong frame/ID')
        return message, max(0.0, age)

    def inputs(self, sim, wall):
        entry = self.m.states.get(self.vehicle_id)
        status, _ = self.sample(entry, sim, wall)
        source_age = status.status_age_s
        if not math.isfinite(source_age) or source_age < 0.0:
            raise ValueError('Invalid leader status age')
        if not status.status_valid or source_age + wall - entry[1] > 1.5:
            raise StatusNotFresh('Leader PX4 status is not fresh')
        if not status.armed or status.failsafe:
            raise ValueError('Leader is disarmed or in failsafe')
        if (status.nav_state not in (4, self.external)
                or status.nav_state_user_intention not in (4, self.external)):
            raise ValueError('Leader flight mode changed')
        if self.seen_external and (
            status.nav_state != self.external
            or status.nav_state_user_intention != self.external
        ):
            raise ValueError('Leader left Swarm Control')

        mesh, age = self.sample(self.mesh, sim, wall)
        values = (mesh.position.x, mesh.position.y, mesh.position.z,
                  mesh.velocity.x, mesh.velocity.y, mesh.velocity.z)
        if not (
            all(math.isfinite(v) for v in values)
            and mesh.status in (MeshEstimate.VALID, MeshEstimate.DEGRADED)
            and 0.2 <= mesh.quality <= 1.0
            and mesh.used_station_count >= 3
            and 0.0 <= mesh.measurement_age_s <= 0.5 - age
        ):
            raise ValueError('Invalid leader mesh estimate')
        ground_speed = math.hypot(mesh.velocity.x, mesh.velocity.y)
        if ground_speed < 5.0:
            raise ValueError('Insufficient leader ground speed')
        if self.running and mesh.reset_counter != self.counter:
            raise ValueError('Leader localization reset')
        altitude = self.origin + mesh.position.z + mesh.velocity.z * age
        course = math.atan2(mesh.velocity.x, mesh.velocity.y)
        return status, mesh, course, altitude, ground_speed

    def start(self):
        if self.running:
            return False, 'Leader command stream already running'
        sim = self.m.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        # An explicit restart in Hold is the only way to clear the fault latch.
        old_seen = self.seen_external
        self.seen_external = False
        try:
            status, mesh, course, altitude, _ = self.inputs(sim, wall)
            if status.nav_state != 4 or status.nav_state_user_intention != 4:
                raise ValueError('Start the leader stream from armed Hold')
            if not self.origin + 20.0 <= altitude <= self.origin + 250.0:
                raise ValueError('Leader height must be 20..250 m above the origin')
            if self.last_sim is None or wall - self.clock_at > 1.0:
                raise ValueError('Wait for an advancing simulation clock')
        except ValueError as error:
            self.seen_external = old_seen
            return False, str(error)
        self.course, self.altitude, self.speed = course, altitude, 15.0
        self.target = (course, altitude, 15.0)
        self.counter = mesh.reset_counter
        self.last_sim, self.clock_at = sim, wall
        self.status_wait_since = None
        self.published_at = 0.0
        self.fault = ''
        self.running = True
        return True, 'Leader stream started at current course and height'

    def stop(self):
        self.running = False
        self.published_at = 0.0
        self.status_wait_since = None

    def fail(self, reason):
        self.stop()
        self.fault = reason
        self.m.get_logger().error(f'Leader command stream stopped: {reason}')

    def healthy(self):
        if not self.running or self.fault:
            return False
        wall = time.monotonic()
        sim = self.m.get_clock().now().nanoseconds * 1e-9
        if self.last_sim is None or sim < self.last_sim or wall - self.clock_at > 1.0:
            return False
        try:
            self.inputs(sim, wall)
            return True
        except ValueError:
            return False

    def set_command(self, request, response):
        response.success = False
        if hasattr(self.m, 'mission') and self.m.mission.owns_control:
            response.message = 'Mission owns leader commands; use Hold or mission/cancel first'
            return response
        values = (request.course_rad, request.altitude_amsl_m,
                  request.equivalent_airspeed_m_s)
        if not all(math.isfinite(v) for v in values):
            response.message = 'Command values must be finite'
        elif not (
            abs(values[0]) <= math.pi + 1e-6
            and self.origin + 20.0 <= values[1] <= self.origin + 250.0
            and 10.0 <= values[2] <= 19.0
        ):
            response.message = 'Use course [-pi, pi], height 20..250 m above origin, EAS 10..19 m/s'
        elif not self.healthy():
            response.message = self.fault or 'Leader stream is not ready'
        else:
            self.target = (math.atan2(math.sin(values[0]), math.cos(values[0])),
                           values[1], values[2])
            response.success = True
            response.message = 'Leader target accepted; bounded transition in progress'
        return response

    @staticmethod
    def approach(value, target, step):
        return value + max(-step, min(step, target - value))

    def tick(self):
        wall = time.monotonic()
        sim = self.m.get_clock().now().nanoseconds * 1e-9
        if self.last_sim is None:
            self.last_sim, self.clock_at = sim, wall
            return
        dt = sim - self.last_sim
        if dt == 0.0:
            if self.running and wall - self.clock_at > 1.0:
                self.fail('Simulation clock stopped')
            return
        self.last_sim, self.clock_at = sim, wall
        if dt < 0.0 or dt > 0.25:
            if self.running:
                self.fail('Simulation time jump')
            if dt < 0.0:
                self.mesh = None
            return
        if not self.running:
            return
        try:
            status, _, course, altitude, ground_speed = self.inputs(sim, wall)
        except StatusNotFresh as error:
            if self.status_wait_since is None:
                self.status_wait_since = wall
            elif wall - self.status_wait_since >= 0.5:
                self.fail(str(error))
            return
        except ValueError as error:
            self.fail(str(error))
            return
        self.status_wait_since = None
        if status.nav_state == self.external and not self.seen_external:
            self.seen_external = True
            self.course, self.altitude = course, altitude
        velocity = max(ground_speed, self.speed, 5.0)
        turn_rate = min(9.80665 * math.tan(math.radians(25.0)) / velocity,
                        velocity / 120.0)
        error = math.atan2(math.sin(self.target[0] - self.course),
                           math.cos(self.target[0] - self.course))
        self.course += max(-turn_rate * dt, min(turn_rate * dt, error))
        self.course = math.atan2(math.sin(self.course), math.cos(self.course))
        self.altitude = self.approach(self.altitude, self.target[1], 1.5 * dt)
        self.speed = self.approach(self.speed, self.target[2], 1.0 * dt)
        stamp = self.system_clock.now()
        if stamp.nanoseconds <= self.last_stamp:
            self.fail('System clock stopped or moved backwards')
            return
        self.last_stamp = stamp.nanoseconds
        command = FixedWingCommand()
        command.stamp = stamp.to_msg()
        command.course_rad = float(self.course)
        command.altitude_amsl_m = float(self.altitude)
        command.equivalent_airspeed_m_s = float(self.speed)
        self.publisher.publish(command)
        self.published_at = wall
