import math
import time

from rclpy.clock import Clock, ClockType
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FixedWingCommand

from .leader_source import LeaderSource


class ActivationError(RuntimeError):
    pass


class FlightCoordinator:
    """Command preparation, checked activation, and supervised group flight."""

    def __init__(self, manager):
        self.m = manager
        manager.declare_parameter('external_nav_state', 23)
        self.external = manager.get_parameter('external_nav_state').value
        if not 23 <= self.external <= 30:
            raise ValueError('external_nav_state must be 23..30')
        self.leader = LeaderSource(manager, self.external)
        self.system = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.engaged = False
        self.bad_since = None
        self.commands = {}
        self.command_subs = []
        self.activate_clients = {}
        self.bridge_resets = {}
        self.follower_resets = {}
        for i in manager.ids:
            topic = f'/plane_{i}/swarm_control/command'
            self.command_subs.append(manager.create_subscription(
                FixedWingCommand, topic,
                lambda msg, i=i: self.commands.update({i: (msg, time.monotonic())}),
                1, callback_group=manager.group,
            ))
            self.activate_clients[i] = manager.create_client(
                Trigger, f'/plane_{i}/flight/activate_swarm', callback_group=manager.group,
            )
            self.bridge_resets[i] = manager.create_client(
                Trigger, f'/plane_{i}/swarm_control/reset_fault', callback_group=manager.group,
            )
            if i != manager.ids[0]:
                self.follower_resets[i] = manager.create_client(
                    Trigger, f'/follower_{i}/reset', callback_group=manager.group,
                )
        self.service = manager.create_service(
            Trigger, '/swarm/activate', self.activate, callback_group=manager.group,
        )
        self.timer = manager.create_timer(
            0.1, self.monitor, clock=manager.steady, callback_group=manager.group,
        )

    def state(self, i):
        return self.m.takeoff_sequence.state(i)

    def status_age(self, i):
        entry = self.m.states.get(i)
        return math.inf if entry is None else entry[0].status_age_s + time.monotonic() - entry[1]

    def mode(self, i, expected):
        s = self.state(i)
        return bool(s is not None and s.armed and not s.failsafe
                    and s.nav_state == expected and s.nav_state_user_intention == expected
                    and self.m.takeoff_sequence.navigation_ready(i, s))

    def streams_ready(self, since=0.0):
        now = time.monotonic()
        wall = self.system.now().nanoseconds * 1e-9
        for i in self.m.ids:
            entry = self.commands.get(i)
            if entry is None:
                return False
            msg, received = entry
            stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if not (
                received >= since and 0.0 <= now - received <= 0.4
                and stamp > 0.0 and -0.1 <= wall - stamp <= 0.5
                and all(math.isfinite(v) for v in (
                    msg.course_rad, msg.altitude_amsl_m, msg.equivalent_airspeed_m_s))
                and abs(msg.course_rad) <= math.pi + 1e-6
                and 10.0 <= msg.equivalent_airspeed_m_s <= 20.0
                and self.m.count_publishers(f'/plane_{i}/swarm_control/command') == 1
            ):
                return False
        return True

    def problem(self, confirmed, requested):
        for i in self.m.ids:
            s = self.state(i)
            if s is None:
                continue  # Bounded waits handle missing telemetry.
            if not s.armed or s.failsafe:
                return f'plane_{i}: disarmed or failsafe'
            allowed = {self.external} if i in confirmed else (
                {4, self.external} if i in requested else {4})
            if s.nav_state not in allowed or s.nav_state_user_intention not in allowed:
                return f'plane_{i}: unexpected mode {s.nav_state}/{s.nav_state_user_intention}'
        return ''

    async def wait(self, predicate, generation, confirmed, requested, timeout=8.0):
        m = self.m
        ok = await m.wait_until(
            lambda: generation != m.generation or self.leader.fault
            or self.problem(confirmed, requested) or predicate(), timeout,
        )
        if generation != m.generation:
            raise ActivationError('Interrupted by Hold or another recovery')
        reason = self.leader.fault or self.problem(confirmed, requested)
        if reason or not ok or not predicate():
            raise ActivationError(reason or 'Readiness or mode confirmation timeout')

    async def call(self, clients, generation):
        results = await self.m.dispatch(clients, Trigger.Request, generation)
        if generation != self.m.generation:
            raise ActivationError('Interrupted by Hold')
        if not all(ok for ok, _ in results.values()):
            raise ActivationError(self.m.report(results))

    def begin_hold(self):
        # Keep a healthy command stream until the actual mode changes.
        self.engaged = False
        self.bad_since = None

    def finish_hold(self):
        s = self.state(self.m.ids[0])
        if s is not None and s.nav_state == 4 and s.nav_state_user_intention == 4:
            self.leader.stop()

    async def recover(self, generation, requested):
        self.engaged = False
        candidates = {}
        for i in requested:
            s = self.state(i)
            if (s is not None and s.armed and not s.failsafe
                    and s.nav_state == self.external
                    and s.nav_state_user_intention == self.external):
                candidates[i] = self.m.hold_clients[i]
        if not candidates:
            self.finish_hold()
            return 'No fresh External aircraft eligible for automatic Hold; verify actual modes'
        started = time.monotonic()
        results = await self.m.dispatch(candidates, Trigger.Request, generation)
        if generation != self.m.generation:
            return 'Recovery interrupted'
        await self.m.wait_until(
            lambda: generation != self.m.generation or all(
                self.m.hold_confirmed(i, started) for i in candidates), self.m.hold_timeout,
        )
        if generation != self.m.generation:
            return 'Recovery interrupted'
        self.finish_hold()
        missing = [i for i in candidates if not self.m.hold_confirmed(i, started)]
        return self.m.report(results) + f'; Hold unconfirmed={missing}'

    async def activate(self, request, response):
        m = self.m
        response.success = False
        if m.operation is not None:
            response.message = f'Manager busy: {m.operation}'
            return response
        groups = (self.activate_clients, self.bridge_resets, self.follower_resets, m.hold_clients)
        if any(not c.service_is_ready() for group in groups for c in group.values()):
            response.message = 'Activation/reset/Hold services unavailable; nothing sent'
            return response
        if not all(self.mode(i, 4) for i in m.ids):
            response.message = 'All aircraft must be airborne, armed and healthy in Hold'
            return response
        m.generation += 1
        generation = m.generation
        m.operation = 'activate'
        confirmed, requested = set(), set()
        self.engaged = False
        try:
            ok, message = self.leader.start()
            if not ok:
                raise ActivationError(message)
            for i, client in self.follower_resets.items():
                await self.wait(
                    lambda i=i: all(self.mode(j, 4) for j in m.ids)
                    and self.status_age(i) < 1.0 and self.status_age(m.ids[0]) < 1.0,
                    generation, confirmed, requested,
                )
                await self.call({i: client}, generation)
            warm_since = time.monotonic()
            await self.wait(
                lambda: time.monotonic() - warm_since >= 0.3
                and self.leader.healthy() and self.streams_ready(warm_since),
                generation, confirmed, requested,
            )
            for i, client in self.bridge_resets.items():
                await self.wait(
                    lambda i=i: all(self.mode(j, 4) for j in m.ids)
                    and self.status_age(i) < 0.25 and self.streams_ready(),
                    generation, confirmed, requested,
                )
                await self.call({i: client}, generation)
            for i, client in self.activate_clients.items():
                await self.wait(
                    lambda i=i: self.leader.healthy() and self.streams_ready()
                    and self.status_age(i) < 0.7
                    and all(self.mode(j, self.external if j in confirmed else 4) for j in m.ids),
                    generation, confirmed, requested,
                )
                requested.add(i)
                await self.call({i: client}, generation)
                await self.wait(
                    lambda i=i: self.mode(i, self.external)
                    and self.leader.healthy() and self.streams_ready()
                    and all(self.mode(j, self.external if j in confirmed else 4)
                            for j in m.ids if j != i),
                    generation, confirmed, requested,
                )
                confirmed.add(i)
                m.get_logger().info(f'plane_{i}: Swarm Control confirmed')
            self.engaged = True
            self.bad_since = None
            response.success = True
            response.message = 'Swarm Control active for all aircraft; formation convergence is not yet confirmed'
        except ActivationError as error:
            response.message = str(error)
            if generation == m.generation:
                response.message += '; recovery: ' + await self.recover(generation, requested)
            response.message += '; requests with unknown results are not repeated'
        finally:
            if generation == m.generation:
                m.operation = None
            m.get_logger().info(response.message)
        return response

    def monitor(self):
        if not self.engaged or self.m.operation in ('hold', 'activate', 'recover'):
            return
        reason = self.leader.fault or self.problem(set(self.m.ids), set(self.m.ids))
        ready = (self.leader.healthy() and self.streams_ready()
                 and all(self.mode(i, self.external) for i in self.m.ids))
        if ready and not reason:
            self.bad_since = None
            return
        now = time.monotonic()
        if not reason:
            if self.bad_since is None:
                self.bad_since = now
            if now - self.bad_since < 0.75:
                return
            reason = 'Group telemetry or command stream did not recover'
        self.engaged = False
        self.m.generation += 1
        generation = self.m.generation
        self.m.operation = 'recover'
        self.m.get_logger().error(reason)
        self.m.executor.create_task(self.recovery_task(generation))

    async def recovery_task(self, generation):
        try:
            result = await self.recover(generation, set(self.m.ids))
            self.m.get_logger().warning(result)
        finally:
            if generation == self.m.generation:
                self.m.operation = None
