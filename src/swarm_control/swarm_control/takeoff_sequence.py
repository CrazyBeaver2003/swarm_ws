import math
import time

from std_srvs.srv import Trigger


class TakeoffSequence:
    """Sequential takeoff, confirmed by telemetry in the shared frame."""

    def __init__(self, manager):
        self.m = manager
        manager.declare_parameter(
            'takeoff_heights_m',
            [70.0 + 35.0 * i for i in range(len(manager.ids))],
        )
        manager.declare_parameter('takeoff_wait_s', 330.0)
        self.heights = tuple(manager.get_parameter('takeoff_heights_m').value)
        self.timeout = manager.get_parameter('takeoff_wait_s').value
        if (
            len(self.heights) != len(manager.ids)
            or not all(math.isfinite(h) and 20.0 <= h <= 250.0
                       for h in self.heights)
            or not math.isfinite(self.timeout)
            or not 30.0 <= self.timeout <= 660.0
        ):
            raise ValueError('Invalid takeoff heights or timeout')

        self.clients = {
            i: manager.create_client(
                Trigger, f'/plane_{i}/flight/takeoff',
                callback_group=manager.group,
            )
            for i in manager.ids
        }
        self.service = manager.create_service(
            Trigger, '/swarm/takeoff', self.run,
            callback_group=manager.group,
        )

    def state(self, vehicle_id, since=0.0):
        entry = self.m.states.get(vehicle_id)
        if entry is None:
            return None
        state, received = entry
        age = time.monotonic() - received
        now = self.m.get_clock().now().nanoseconds * 1e-9
        stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        if not (
            received >= since and 0.0 <= age <= 1.0
            and state.vehicle_id == vehicle_id
            and state.header.frame_id == self.m.frame
            and -0.02 <= now - stamp <= 0.3
            and state.status_valid
            and math.isfinite(state.status_age_s)
            and 0.0 <= state.status_age_s
            and state.status_age_s + age <= 2.0
        ):
            return None
        return state

    def navigation_ready(self, vehicle_id, state):
        age = time.monotonic() - self.m.states[vehicle_id][1]
        return (
            state.position_valid and state.velocity_valid and state.altitude_valid
            and all(math.isfinite(a) and 0.0 <= a and a + age <= 1.0
                    for a in (state.position_age_s, state.velocity_age_s,
                              state.altitude_age_s))
            and all(math.isfinite(v) for v in (
                state.position.x, state.position.y, state.position.z,
                state.velocity.x, state.velocity.y, state.velocity.z,
                state.altitude_amsl_m,
            ))
        )

    @staticmethod
    def in_hold(state):
        return state.nav_state == 4 and state.nav_state_user_intention == 4

    def airborne(self, vehicle_id, state, target):
        return (
            state is not None and not state.failsafe and state.armed
            and self.in_hold(state) and self.navigation_ready(vehicle_id, state)
            and abs(state.altitude_amsl_m - target) <= 5.0
            and math.hypot(state.velocity.x, state.velocity.y) >= 5.0
        )

    def ready_on_ground(self, vehicle_id):
        state = self.state(vehicle_id)
        return (
            state is not None and not state.failsafe and not state.armed
            and self.in_hold(state) and self.navigation_ready(vehicle_id, state)
        )

    async def run(self, request, response, *, _mission=False):
        m = self.m
        response.success = False
        if hasattr(m, 'mission') and m.mission.owns_control and not _mission:
            response.message = 'Mission owns flight control; use Hold to cancel'
            return response
        if m.operation is not None:
            response.message = f'Manager busy: {m.operation}'
            return response
        unavailable = [i for i, c in self.clients.items() if not c.service_is_ready()]
        unready = [i for i in m.ids if not self.ready_on_ground(i)]
        if unavailable or unready:
            response.message = (
                f'Nothing sent: unavailable={unavailable}, '
                f'not ready in disarmed Hold={unready}'
            )
            return response

        m.generation += 1
        generation = m.generation
        m.operation = 'takeoff'
        targets = {}
        current = None
        try:
            for current, height in zip(m.ids, self.heights):
                if generation != m.generation:
                    response.message = 'Takeoff queue interrupted by Hold'
                    return response
                # Brief gaps in VM telemetry must not trigger another launch.
                def group_ready():
                    return all(
                        self.airborne(i, self.state(i), targets[i]) if i in targets
                        else self.ready_on_ground(i)
                        for i in m.ids
                    )

                ready = await m.wait_until(
                    lambda: generation != m.generation or group_ready(), 3.0,
                )
                if generation != m.generation:
                    response.message = 'Takeoff queue interrupted by Hold'
                    return response
                state = self.state(current)
                if not ready or state is None or not group_ready():
                    response.message = f'Queue stopped before plane_{current}: group not ready'
                    return response
                targets[current] = state.altitude_amsl_m + height
                sent = time.monotonic()
                m.get_logger().info(
                    f'Takeoff plane_{current}: target {targets[current]:.1f} m AMSL'
                )
                results = await m.dispatch(
                    {current: self.clients[current]}, Trigger.Request, generation,
                )
                if generation != m.generation:
                    response.message = 'Takeoff queue interrupted by Hold'
                    return response
                if not results[current][0]:
                    response.message = m.report(results)
                    return response

                last_good = {i: time.monotonic() for i in m.ids}
                error = []
                seen_armed = False

                def progress():
                    nonlocal seen_armed
                    if generation != m.generation:
                        return True
                    all_ready = True
                    now = time.monotonic()
                    for i in m.ids:
                        state = self.state(i, sent)
                        fresh = state is not None
                        if fresh:
                            if state.failsafe:
                                error.append(f'plane_{i}: failsafe')
                                return True
                            if i == current:
                                seen_armed = seen_armed or state.armed
                                if seen_armed and not state.armed:
                                    error.append(f'plane_{i}: disarmed during takeoff')
                                    return True
                                if (state.nav_state not in (4, 17)
                                        or state.nav_state_user_intention not in (4, 17)):
                                    error.append(f'plane_{i}: flight mode changed')
                                    return True
                            elif i in targets:
                                if not state.armed or not self.in_hold(state):
                                    error.append(f'plane_{i}: left armed Hold')
                                    return True
                            elif state.armed or not self.in_hold(state):
                                error.append(f'plane_{i}: pending aircraft state changed')
                                return True
                            fresh = self.navigation_ready(i, state)
                        if not fresh:
                            all_ready = False
                            if now - last_good[i] > 3.0:
                                error.append(f'plane_{i}: telemetry did not recover')
                                return True
                            continue
                        last_good[i] = now
                        if i in targets and not self.airborne(i, state, targets[i]):
                            all_ready = False
                    return all_ready

                finished = await m.wait_until(progress, self.timeout)
                if generation != m.generation:
                    response.message = 'Takeoff queue interrupted by Hold'
                    return response
                if error or not finished:
                    response.message = error[0] if error else f'plane_{current}: takeoff confirmation timeout'
                    return response
                m.get_logger().info(f'plane_{current}: airborne in Hold; continuing queue')

            response.success = True
            response.message = 'All aircraft airborne at assigned heights in Hold'
            return response
        finally:
            if generation == m.generation:
                m.operation = None
            if not response.success:
                response.message += (
                    '; no further takeoff requests will be sent. '
                    'Previously sent requests may still execute; check aircraft state'
                )
            m.get_logger().info(response.message)
