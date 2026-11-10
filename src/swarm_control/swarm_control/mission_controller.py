"""Operator goal -> takeoff, assembly, bounded-curvature route, confirmed Hold."""
import json
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import MeshEstimate
from swarm_interfaces.srv import SetMissionGoal

from .mission_core import Pose2, RouteFollower, plan_mission


class MissionError(RuntimeError):
    pass


class MissionController:
    def __init__(self, manager):
        self.m = manager
        defaults = {
            'mission_height_m': 70.0,
            'mission_turn_radius_m': 350.0,
            'mission_airspeed_m_s': 15.0,
            'mission_lookahead_m': 70.0,
            'mission_assembly_tolerance_m': 20.0,
            'mission_assembly_settle_s': 3.0,
            'mission_assembly_timeout_s': 360.0,
            'mission_route_timeout_s': 900.0,
        }
        for name, value in defaults.items():
            manager.declare_parameter(name, value)
        self.height = manager.get_parameter('mission_height_m').value
        self.radius = manager.get_parameter('mission_turn_radius_m').value
        self.speed = manager.get_parameter('mission_airspeed_m_s').value
        self.lookahead = manager.get_parameter('mission_lookahead_m').value
        self.tolerance = manager.get_parameter('mission_assembly_tolerance_m').value
        self.settle = manager.get_parameter('mission_assembly_settle_s').value
        self.assembly_timeout = manager.get_parameter('mission_assembly_timeout_s').value
        self.route_timeout = manager.get_parameter('mission_route_timeout_s').value
        values = (self.height, self.radius, self.speed, self.lookahead,
                  self.tolerance, self.settle, self.assembly_timeout, self.route_timeout)
        if not (all(math.isfinite(v) for v in values)
                and 40.0 <= self.height <= 250.0-35.0*(len(manager.ids)-1)
                and 120.0 <= self.radius <= 2000.0 and 10.0 <= self.speed <= 19.0
                and 20.0 <= self.lookahead <= 300.0 and 5.0 <= self.tolerance <= 50.0
                and 0.5 <= self.settle <= 15.0 and 10.0 <= self.assembly_timeout <= 1200.0
                and 30.0 <= self.route_timeout <= 3600.0):
            raise ValueError('Invalid mission height, geometry or timeouts')
        self.mesh = {}
        self.targets = {}
        self.subscriptions = []
        for i in manager.ids:
            self.subscriptions.append(manager.create_subscription(
                MeshEstimate, f'/plane_{i}/mesh/estimate',
                lambda msg, i=i: self.mesh.update({i: (msg, time.monotonic())}),
                qos_profile_sensor_data, callback_group=manager.group,
            ))
            if i != manager.ids[0]:
                self.subscriptions.append(manager.create_subscription(
                    PointStamped, f'/follower_{i}/target',
                    lambda msg, i=i: self.targets.update({i: (msg, time.monotonic())}),
                    qos_profile_sensor_data, callback_group=manager.group,
                ))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = manager.create_publisher(Path, '/swarm/mission/path', latched)
        self.status_pub = manager.create_publisher(String, '/swarm/mission/status', latched)
        self.goal_service = manager.create_service(
            SetMissionGoal, '/swarm/mission/set_goal', self.set_goal,
            callback_group=manager.group,
        )
        self.cancel_service = manager.create_service(
            Trigger, '/swarm/mission/cancel', self.cancel_service_callback,
            callback_group=manager.group,
        )
        self.owns_control = False
        self.token = 0
        self.mission_id = 0
        self.phase = 'idle'
        self.detail = 'Waiting for a map goal'
        self.goal = None
        self.arrival_radius = 25.0
        self.start_sim = None
        self.reached_sim = None
        self.task = None
        self.status_timer = manager.create_timer(
            0.5, self.publish_status, clock=manager.steady, callback_group=manager.group,
        )
        self.publish_status()

    def sim_time(self):
        return self.m.get_clock().now().nanoseconds * 1e-9

    @property
    def altitude(self):
        return self.m.flight.leader.origin + self.height

    def publish_status(self):
        elapsed = None if self.start_sim is None else max(0.0, self.sim_time()-self.start_sim)
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase, 'mission_id': self.mission_id, 'goal': self.goal,
            'goal_east_m': None if self.goal is None else self.goal[0],
            'goal_north_m': None if self.goal is None else self.goal[1],
            'altitude_amsl_m': self.altitude,
            'working_altitude_amsl_m': self.altitude,
            'start_sim_s': self.start_sim, 'reached_sim_s': self.reached_sim,
            'elapsed_s': elapsed, 'reached': self.reached_sim is not None,
            'owns_control': self.owns_control, 'detail': self.detail,
        }, allow_nan=False)))

    def stage(self, phase, detail):
        self.phase, self.detail = phase, detail
        self.publish_status()
        self.m.get_logger().info(f'Mission {self.mission_id}: {phase}: {detail}')

    def cancel(self, reason='Operator requested Hold'):
        if self.owns_control:
            self.token += 1
            self.stage('cancelled', reason)

    async def cancel_service_callback(self, request, response):
        response.success = False
        if not self.owns_control:
            response.message = 'No running mission'
            return response
        self.cancel('Mission cancellation requested')
        result = await self.m.hold(Trigger.Request(), Trigger.Response(), _mission=True)
        response.success = result.success
        response.message = f'Mission cancelled; Hold: {result.message}'
        return response

    def sample(self, vehicle_id):
        entry = self.mesh.get(vehicle_id)
        if entry is None:
            raise MissionError(f'plane_{vehicle_id}: no mesh estimate')
        msg, received = entry
        age = self.sim_time() - (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        if not (
            msg.vehicle_id == vehicle_id and msg.header.frame_id == self.m.frame
            and -0.02 <= age <= 0.3 and 0.0 <= time.monotonic()-received <= 1.0
            and msg.status in (MeshEstimate.VALID, MeshEstimate.DEGRADED)
            and 0.2 <= msg.quality <= 1.0 and msg.used_station_count >= 3
            and 0.0 <= msg.measurement_age_s <= 0.5-max(age, 0.0)
            and all(math.isfinite(v) for v in (
                msg.position.x, msg.position.y, msg.position.z,
                msg.velocity.x, msg.velocity.y, msg.velocity.z))
        ):
            raise MissionError(f'plane_{vehicle_id}: mesh estimate not ready')
        return msg

    def publish_route(self, route):
        msg = Path()
        msg.header.frame_id = self.m.frame
        msg.header.stamp = self.m.get_clock().now().to_msg()
        for point in route.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = self.height
            pose.pose.orientation.z = math.sin(point.yaw/2.0)
            pose.pose.orientation.w = math.cos(point.yaw/2.0)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def set_goal(self, request, response):
        response.success = False
        if self.owns_control or self.m.operation is not None:
            response.message = 'Manager or mission busy; cancel or Hold first'
            return response
        if not (all(math.isfinite(v) and abs(v) < 10000.0
                    for v in (request.east_m, request.north_m))
                and math.isfinite(request.arrival_radius_m)
                and 10.0 <= request.arrival_radius_m <= 100.0):
            response.message = 'Goal ENU within +/-10000 m; arrival radius 10..100 m'
            return response
        grounded = all(self.m.takeoff_sequence.ready_on_ground(i) for i in self.m.ids)
        airborne = all(self.m.flight.mode(i, 4) for i in self.m.ids)
        if not grounded and not airborne:
            response.message = 'Need the entire group disarmed on ground or armed in Hold'
            return response
        try:
            self.sample(self.m.ids[0])
        except MissionError as error:
            response.message = str(error)
            return response
        self.goal = (float(request.east_m), float(request.north_m))
        self.arrival_radius = float(request.arrival_radius_m)
        self.start_sim, self.reached_sim = self.sim_time(), None
        self.token += 1
        self.mission_id += 1
        token = self.token
        self.owns_control = True
        self.stage('planning', 'Flat obstacle-free map; height chosen from configuration')
        self.task = self.m.executor.create_task(self.run(token, grounded))
        response.success = True
        response.message = f'Mission {self.mission_id} accepted; observe /swarm/mission/status'
        return response

    def cancelled(self, token):
        return token != self.token

    def group_ready(self):
        if not self.m.flight.engaged or self.m.flight.leader.fault:
            raise MissionError(self.m.flight.leader.fault or 'Swarm Control is no longer active')
        for i in self.m.ids:
            if not self.m.flight.mode(i, self.m.flight.external):
                return False
            self.sample(i)
        return True

    def formation_ready(self):
        if not self.group_ready():
            return False
        for i in self.m.ids[1:]:
            entry = self.targets.get(i)
            if entry is None:
                return False
            target, received = entry
            age = self.sim_time()-(target.header.stamp.sec + target.header.stamp.nanosec*1e-9)
            if not (target.header.frame_id == self.m.frame and -0.02 <= age <= 0.3
                    and time.monotonic()-received <= 1.0
                    and all(math.isfinite(v) for v in (target.point.x, target.point.y, target.point.z))):
                return False
            p = self.sample(i).position
            if math.sqrt((p.x-target.point.x)**2 + (p.y-target.point.y)**2
                         + (p.z-target.point.z)**2) > self.tolerance:
                return False
        p = self.sample(self.m.ids[0]).position
        return abs(p.z-self.height) <= 5.0

    async def wait_flight(self, token, predicate, timeout):
        errors = []
        bad_since = None

        def poll():
            nonlocal bad_since
            if self.cancelled(token):
                return True
            try:
                if not self.group_ready():
                    if bad_since is None:
                        bad_since = time.monotonic()
                    if time.monotonic()-bad_since > 1.0:
                        raise MissionError('Group state did not recover')
                    return False
                bad_since = None
                return predicate()
            except (MissionError, ValueError) as error:
                errors.append(str(error))
                return True
        finished = await self.m.wait_until(poll, timeout)
        if self.cancelled(token):
            return False
        if errors or not finished:
            raise MissionError(errors[0] if errors else 'Mission stage timeout')
        return True

    async def run(self, token, grounded):
        finishing = False
        try:
            leader = self.sample(self.m.ids[0])
            yaw = math.atan2(leader.velocity.y, leader.velocity.x)
            preview = plan_mission(Pose2(leader.position.x, leader.position.y, yaw),
                                   self.goal, self.height, self.radius)
            self.publish_route(preview)
            if grounded:
                self.stage('takeoff', 'Sequential takeoff of the entire group')
                result = await self.m.takeoff_sequence.run(
                    Trigger.Request(), Trigger.Response(), _mission=True)
                if self.cancelled(token):
                    return
                if not result.success:
                    raise MissionError(result.message)
            self.stage('activate', 'Preparing command streams and checking each external mode')
            result = await self.m.flight.activate(Trigger.Request(), Trigger.Response(), _mission=True)
            if self.cancelled(token):
                return
            if not result.success:
                raise MissionError(result.message)
            source = self.m.flight.leader
            source.target = (source.course, self.altitude, self.speed)
            self.stage('assembling', 'Waiting for sustained follower slot convergence')
            stable_since = None

            def assembled():
                nonlocal stable_since
                if not self.formation_ready():
                    stable_since = None
                    return False
                if stable_since is None:
                    stable_since = self.sim_time()
                return self.sim_time()-stable_since >= self.settle
            if not await self.wait_flight(token, assembled, self.assembly_timeout):
                return
            leader = self.sample(self.m.ids[0])
            start = Pose2(leader.position.x, leader.position.y,
                          math.atan2(leader.velocity.y, leader.velocity.x))
            route = plan_mission(start, self.goal, self.height, self.radius)
            tracker = RouteFollower(route, self.arrival_radius)
            self.publish_route(route)
            self.stage('cruise', f'{route.word}, length={route.length:.1f} m, radius={route.radius:.1f} m')
            last_sim = self.sim_time()

            def fly():
                nonlocal last_sim
                sim = self.sim_time()
                dt = sim-last_sim
                if dt == 0.0:
                    return False
                if not 0.0 < dt <= 0.5:
                    raise MissionError('Simulation clock jumped during route tracking')
                last_sim = sim
                p = self.sample(self.m.ids[0])
                course = tracker.desired_course(
                    p.position.x, p.position.y,
                    math.hypot(p.velocity.x, p.velocity.y), dt, self.lookahead)
                source.target = (course, self.altitude, self.speed)
                if tracker.reached and abs(p.position.z-self.height) <= 5.0:
                    self.reached_sim = sim
                    return True
                return False
            if not await self.wait_flight(token, fly, self.route_timeout):
                return
            self.stage('finishing', 'Goal reached; requesting and confirming Hold for all aircraft')
            finishing = True
            result = await self.m.hold(Trigger.Request(), Trigger.Response(), _mission=True)
            if self.cancelled(token):
                return
            if not result.success:
                raise MissionError('Goal reached but group Hold not confirmed: '+result.message)
            self.stage('completed', 'Goal reached; the entire group is in Hold')
        except (MissionError, ValueError) as error:
            if not self.cancelled(token):
                self.stage('failed', str(error))
                if not finishing and self.m.operation not in ('hold', 'recover'):
                    self.m.generation += 1
                    generation = self.m.generation
                    self.m.operation = 'recover'
                    try:
                        await self.m.flight.recover(generation, set(self.m.ids))
                    finally:
                        if generation == self.m.generation:
                            self.m.operation = None
        finally:
            self.owns_control = False
            self.publish_status()
