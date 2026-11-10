import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from std_srvs.srv import Trigger
from swarm_interfaces.msg import VehicleState
from swarm_interfaces.srv import SetFormation
from swarm_control.takeoff_sequence import TakeoffSequence
from swarm_control.flight_coordinator import FlightCoordinator

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')

        self.declare_parameter('vehicle_ids', [1, 2, 3])
        self.declare_parameter('frame_id', 'swarm_map')
        self.declare_parameter('rpc_timeout_s', 3.0)
        self.declare_parameter('hold_timeout_s', 5.0)

        self.ids = tuple(self.get_parameter('vehicle_ids').value)
        self.frame = self.get_parameter('frame_id').value
        self.rpc_timeout = self.get_parameter('rpc_timeout_s').value
        self.hold_timeout = self.get_parameter('hold_timeout_s').value

        if not (
            2 <= len(self.ids) <= 5
            and len(set(self.ids)) == len(self.ids)
            and all(1 <= i <= 255 for i in self.ids)
            and self.frame
        ):
            raise ValueError(
                'Need 2..5 unique vehicle IDs and a frame'
            )

        if not all(
            math.isfinite(t) and 0.0 < t <= 30.0
            for t in (self.rpc_timeout, self.hold_timeout)
        ):
            raise ValueError(
                'Timeouts must be in (0, 30] seconds'
            )

        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Run with use_sim_time:=true')

        self.group = ReentrantCallbackGroup()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)

        self.operation = None
        self.generation = 0

        self.states = {}
        self.subscriptions_ = []
        self.formation_clients = {}
        self.hold_clients = {}

        for vehicle_id in self.ids:
            self.subscriptions_.append(
                self.create_subscription(
                    VehicleState,
                    f'/plane_{vehicle_id}/state',
                    lambda msg, i=vehicle_id: self.states.update({
                        i: (msg, time.monotonic())
                    }),
                    qos_profile_sensor_data,
                    callback_group=self.group,
                )
            )

            self.hold_clients[vehicle_id] = self.create_client(
                Trigger,
                f'/plane_{vehicle_id}/flight/hold',
                callback_group=self.group,
            )

            # Первый ID в списке — лидер.
            if vehicle_id != self.ids[0]:
                self.formation_clients[vehicle_id] = (
                    self.create_client(
                        SetFormation,
                        f'/follower_{vehicle_id}/set_formation',
                        callback_group=self.group,
                    )
                )

        self.formation_service = self.create_service(
            SetFormation,
            '/swarm/set_formation',
            self.set_formation,
            callback_group=self.group,
        )

        self.hold_service = self.create_service(
            Trigger,
            '/swarm/hold',
            self.hold,
            callback_group=self.group,
        )

        self.get_logger().info(
            f'Manager ready: leader={self.ids[0]}, '
            f'followers={self.ids[1:]}'
        )
        
        self.takeoff_sequence = TakeoffSequence(self)
        self.flight = FlightCoordinator(self)

    async def wait_until(self, predicate, timeout):
        result = Future(executor=self.executor)
        deadline = time.monotonic() + timeout

        def poll():
            if result.done() or result.cancelled():
                return

            if predicate():
                result.set_result(True)
            elif time.monotonic() >= deadline:
                result.set_result(False)

        timer = self.create_timer(
            0.05,
            poll,
            clock=self.steady,
            callback_group=self.group,
        )

        try:
            poll()
            return await result
        finally:
            self.destroy_timer(timer)

    async def dispatch(self, clients, make_request, generation):
        futures = {}
        results = {}

        for vehicle_id, client in clients.items():
            if not client.service_is_ready():
                results[vehicle_id] = (
                    False,
                    'service unavailable',
                )
                continue

            try:
                futures[vehicle_id] = client.call_async(
                    make_request()
                )
            except Exception as error:
                results[vehicle_id] = (False, str(error))

        try:
            await self.wait_until(
                lambda: (
                    generation != self.generation
                    or all(
                        future.done() or future.cancelled()
                        for future in futures.values()
                    )
                ),
                self.rpc_timeout,
            )

            for vehicle_id, future in futures.items():
                if future.cancelled() or not future.done():
                    results[vehicle_id] = (
                        False,
                        'no response; remote result unknown',
                    )
                    continue

                try:
                    reply = future.result()
                    results[vehicle_id] = (
                        bool(reply.success),
                        reply.message,
                    )
                except Exception as error:
                    results[vehicle_id] = (False, str(error))

            return results

        finally:
            for future in futures.values():
                if not future.done() and not future.cancelled():
                    # Отменяется местное ожидание.
                    # Удалённая команда могла уже выполниться.
                    future.cancel()

    @staticmethod
    def report(results):
        return '; '.join(
            f'plane_{i}: {"OK" if ok else "FAILED"}: {message}'
            for i, (ok, message) in sorted(results.items())
        )

    async def set_formation(self, request, response):
        response.success = False

        if (
            request.formation not in ('line', 'wedge')
            or not math.isfinite(request.spacing_m)
            or not 45.0 <= request.spacing_m <= 120.0
        ):
            response.message = (
                'Use line/wedge and spacing_m 45..120'
            )
            return response

        if self.operation is not None:
            response.message = (
                f'Manager busy: {self.operation}'
            )
            return response

        missing = [
            i
            for i, client in self.formation_clients.items()
            if not client.service_is_ready()
        ]

        if missing:
            response.message = (
                f'Follower services unavailable: {missing}; '
                'nothing sent'
            )
            return response

        self.generation += 1
        generation = self.generation
        self.operation = 'formation'

        try:
            results = await self.dispatch(
                self.formation_clients,
                lambda: SetFormation.Request(
                    formation=request.formation,
                    spacing_m=request.spacing_m,
                ),
                generation,
            )

            if generation != self.generation:
                response.message = (
                    'Interrupted by Hold; some formation '
                    'requests may have been applied'
                )
            else:
                response.success = all(
                    ok for ok, _ in results.values()
                )
                response.message = self.report(results)

            self.get_logger().info(response.message)
            return response

        finally:
            if generation == self.generation:
                self.operation = None

    def hold_confirmed(self, vehicle_id, started):
        entry = self.states.get(vehicle_id)

        if entry is None:
            return False

        state, received = entry
        receipt_age = time.monotonic() - received

        sim = self.get_clock().now().nanoseconds * 1e-9
        header_age = sim - (
            state.header.stamp.sec
            + state.header.stamp.nanosec * 1e-9
        )

        return (
            received >= started
            and 0.0 <= receipt_age <= 1.0
            and state.vehicle_id == vehicle_id
            and state.header.frame_id == self.frame
            and -0.02 <= header_age <= 0.3
            and state.status_valid
            and math.isfinite(state.status_age_s)
            and state.status_age_s >= 0.0
            and state.status_age_s + receipt_age <= 2.0
            and state.nav_state == 4
            and state.nav_state_user_intention == 4
            and not state.failsafe
        )

    async def hold(self, request, response):
        response.success = False

        if self.operation == 'hold':
            response.message = 'Hold operation already running'
            return response

        # Hold имеет приоритет над ожиданием перестроения.
        self.generation += 1
        generation = self.generation
        self.operation = 'hold'
        self.flight.begin_hold()

        started = time.monotonic()

        try:
            results = await self.dispatch(
                self.hold_clients,
                Trigger.Request,
                generation,
            )

            confirmed = await self.wait_until(
                lambda: all(
                    self.hold_confirmed(i, started)
                    for i in self.ids
                ),
                self.hold_timeout,
            )

            missing = [
                i for i in self.ids
                if not self.hold_confirmed(i, started)
            ]

            detailed = {
                i: (
                    ok and i not in missing,
                    f'request={"accepted" if ok else "failed"}: '
                    f'{message}; '
                    f'Hold={"confirmed" if i not in missing else "not confirmed"}',
                )
                for i, (ok, message) in results.items()
            }

            response.success = confirmed and all(
                ok for ok, _ in detailed.values()
            )

            response.message = self.report(detailed)
            response.message += (
                f'; Hold not confirmed for: {missing}'
            )

            self.get_logger().info(response.message)
            return response

        finally:
            if generation == self.generation:
                self.flight.finish_hold()
                self.operation = None


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SwarmManager()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()