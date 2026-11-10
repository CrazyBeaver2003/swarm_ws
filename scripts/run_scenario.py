#!/usr/bin/env python3
"""Explicit operator-run demonstration on an already running SITL stack.

No simulation process is launched here. Each service request is sent once.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import VehicleState
from swarm_interfaces.srv import SetFormation, SetLeaderCommand, SetMissionGoal


class ScenarioError(RuntimeError):
    pass


class Runner(Node):
    def __init__(self, args, log):
        super().__init__('swarm_scenario_runner', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args, self.log = args, log
        self.ids = tuple(range(1, args.count + 1))
        self.sim = None
        self.clock_received = None
        self.clock_changed = None
        self.states = {}
        self.mission = {}
        self.mission_received = 0.0
        self.started = False
        self.deadline = time.monotonic() + args.wall_timeout
        self.service_clients = {}
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        for i in self.ids:
            self.create_subscription(VehicleState, f'/plane_{i}/state',
                                     lambda msg, i=i: self.states.update({i: (msg, time.monotonic())}),
                                     qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/swarm/mission/status', self.on_mission, latched)

    def event(self, kind, **data):
        payload = {'event': kind, 'wall_utc': datetime.now(timezone.utc).isoformat(),
                   'monotonic_s': time.monotonic(), 'sim_s': self.sim, **data}
        self.log.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + '\n')
        self.log.flush()

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.monotonic()
        if self.sim is not None and value < self.sim:
            self.event('clock_rollback', previous=self.sim, current=value)
            self.deadline = now
        if self.sim is None or value != self.sim:
            self.clock_changed = now
        self.sim, self.clock_received = value, now

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except (TypeError, ValueError):
            return
        if (data.get('mission_id'), data.get('phase')) != (
                self.mission.get('mission_id'), self.mission.get('phase')):
            self.event('mission_phase', status=data)
            print(f"Mission {data.get('mission_id')}: {data.get('phase')} — {data.get('detail', '')}", flush=True)
        self.mission, self.mission_received = data, time.monotonic()

    def wait(self, predicate, timeout, description, emergency=False):
        deadline = time.monotonic() + timeout
        if not emergency:
            deadline = min(deadline, self.deadline)
        while rclpy.ok():
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise ScenarioError(f'Timeout: {description}')
            rclpy.spin_once(self, timeout_sec=0.05)
        raise ScenarioError('ROS context stopped')

    def fresh_state(self, i):
        entry = self.states.get(i)
        if entry is None or self.sim is None:
            return None
        s, received = entry
        receipt_age = time.monotonic() - received
        header_age = self.sim - (s.header.stamp.sec + s.header.stamp.nanosec * 1e-9)
        if not (s.vehicle_id == i and s.header.frame_id == self.args.frame
                and 0 <= receipt_age <= 1.0 and -0.02 <= header_age <= 0.3
                and s.status_valid and math.isfinite(s.status_age_s)
                and 0 <= s.status_age_s and s.status_age_s + receipt_age <= 2.0):
            return None
        return s

    def initial_check(self):
        self.wait(lambda: self.sim is not None and all(self.fresh_state(i) is not None for i in self.ids),
                  45.0, 'initial telemetry')
        initial_clock = self.sim
        self.wait(lambda: self.sim is not None and self.sim > initial_clock, 5.0, 'advancing /clock')
        bad = []
        for i in self.ids:
            s = self.fresh_state(i)
            if s is None or s.armed or s.failsafe or s.nav_state != 4 or s.nav_state_user_intention != 4:
                bad.append(i)
        if bad:
            raise ScenarioError(f'Initial state rejected for {bad}: require the entire group disarmed in Hold. Nothing sent.')
        if self.mission.get('owns_control'):
            raise ScenarioError('A mission already owns control. Nothing sent.')
        self.event('initial_state_confirmed', vehicle_ids=list(self.ids))

    def call(self, name, service=Trigger, fields=None, timeout=20.0, emergency=False):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(service, name)
        client = self.service_clients[name]
        self.wait(client.service_is_ready, 8.0, f'service {name}', emergency=emergency)
        request = service.Request()
        for key, value in (fields or {}).items():
            setattr(request, key, value)
        self.event('request_sent', service=name, request=fields or {})
        future = client.call_async(request)
        if name in ('/swarm/takeoff', '/swarm/mission/set_goal'):
            self.started = True
        try:
            self.wait(future.done, timeout, f'response from {name}', emergency=emergency)
        except ScenarioError:
            future.cancel()
            self.event('response_unknown', service=name, note='Remote request may still execute; no retry')
            raise
        try:
            reply = future.result()
        except Exception as error:
            raise ScenarioError(f'{name}: {error}') from error
        self.event('response', service=name, success=bool(reply.success), message=reply.message)
        print(f'{name}: {reply.success} — {reply.message}', flush=True)
        if not reply.success:
            raise ScenarioError(f'{name}: {reply.message}')
        return reply

    def healthy_group(self):
        values = [self.fresh_state(i) for i in self.ids]
        if any(s is not None and (s.failsafe or not s.armed or
               s.nav_state != self.args.external or s.nav_state_user_intention != self.args.external)
               for s in values):
            raise ScenarioError('Aircraft left the expected armed Swarm Control mode')
        return all(s is not None for s in values)

    def dwell(self, seconds):
        self.wait(lambda: self.sim is not None, 3.0, '/clock')
        target = self.sim + seconds
        stale_since = None
        deadline = min(self.deadline, time.monotonic() + max(30.0, seconds * 12.0))
        while self.sim < target:
            now = time.monotonic()
            if now >= deadline:
                raise ScenarioError('Simulation-time dwell exceeded its wall timeout')
            if self.clock_changed is None or now - self.clock_changed > 3.0:
                raise ScenarioError('/clock stopped')
            if self.healthy_group():
                stale_since = None
            elif stale_since is None:
                stale_since = now
            elif now - stale_since > 3.0:
                raise ScenarioError('Group telemetry did not recover')
            rclpy.spin_once(self, timeout_sec=0.05)
        self.event('dwell_complete', duration_sim_s=seconds)

    def leader(self, course_deg, height_above_origin, speed):
        angle = math.radians(course_deg)
        self.call('/swarm/leader/set_command', SetLeaderCommand, {
            'course_rad': math.atan2(math.sin(angle), math.cos(angle)),
            'altitude_amsl_m': self.args.origin_alt + height_above_origin,
            'equivalent_airspeed_m_s': float(speed),
        })

    def run(self):
        self.initial_check()
        if self.args.scenario == 'mission':
            previous = self.mission.get('mission_id')
            sent = time.monotonic()
            self.call('/swarm/mission/set_goal', SetMissionGoal, {
                'east_m': self.args.east, 'north_m': self.args.north,
                'arrival_radius_m': self.args.arrival_radius,
            })
            self.wait(lambda: self.mission_received >= sent and self.mission.get('mission_id') != previous
                      and self.mission.get('phase') in ('completed', 'failed', 'cancelled'),
                      self.args.wall_timeout, 'final mission status')
            if self.mission['phase'] != 'completed':
                raise ScenarioError(f"Mission ended as {self.mission['phase']}: {self.mission.get('detail', '')}")
            self.event('mission_completed', status=self.mission)
        else:
            self.call('/swarm/takeoff', timeout=self.args.count * 330.0 + 30.0)
            self.call('/swarm/activate', timeout=90.0)
            self.dwell(self.args.settle)
            if self.args.scenario == 'formation':
                for formation, spacing in (('line', 60.0), ('line', 90.0), ('wedge', 90.0), ('line', 60.0)):
                    self.call('/swarm/set_formation', SetFormation,
                              {'formation': formation, 'spacing_m': spacing})
                    self.dwell(self.args.dwell)
                self.leader(90.0, 90.0, 15.0)
                self.dwell(self.args.dwell)
                self.leader(90.0, 70.0, 15.0)
                self.dwell(self.args.dwell)
            else:
                for course, height, speed in ((0.0, 70.0, 12.0), (90.0, 70.0, 17.0),
                                              (45.0, 95.0, 15.0), (90.0, 70.0, 15.0)):
                    self.leader(course, height, speed)
                    self.dwell(self.args.dwell)
            self.call('/swarm/hold')
        # Mission completion already confirms Hold. Capture a final explicit Hold
        # for consistent terminal state even when completion policy later changes.
        if self.args.scenario == 'mission':
            self.call('/swarm/hold')
        self.call('/swarm/evaluation/save', timeout=30.0)
        self.event('scenario_commands_completed', scenario=self.args.scenario,
                   note='Trajectory/formation metrics still require evaluation; timed dwells do not prove convergence')


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=('formation', 'manual', 'mission'))
    parser.add_argument('--count', type=int, choices=(3, 4, 5), default=3)
    parser.add_argument('--frame', default='swarm_map')
    parser.add_argument('--external', type=int, choices=range(23, 31), default=23)
    parser.add_argument('--origin-alt', type=float, default=0.0, help='Common reference altitude, metres AMSL')
    parser.add_argument('--east', type=float, default=1200.0)
    parser.add_argument('--north', type=float, default=800.0)
    parser.add_argument('--arrival-radius', type=float, default=40.0)
    parser.add_argument('--settle', type=float, default=60.0, help='Initial formation wait, simulation seconds')
    parser.add_argument('--dwell', type=float, default=35.0, help='Wait after each change, simulation seconds')
    parser.add_argument('--wall-timeout', type=float, default=3000.0)
    parser.add_argument('--output', type=Path, help='New directory for commands.jsonl; must not already exist')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.origin_alt, args.east, args.north, args.arrival_radius,
                                        args.settle, args.dwell, args.wall_timeout)):
        parser.error('All numeric values must be finite')
    if not (10 <= args.arrival_radius <= 100 and 0 < args.settle <= 600 and 0 < args.dwell <= 600
            and 30 <= args.wall_timeout <= 7200 and abs(args.east) <= 10000 and abs(args.north) <= 10000):
        parser.error('Invalid goal, radius or timeout')
    return args


def main():
    args = arguments()
    directory = args.output or Path('runs') / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ_') + args.scenario)
    directory.mkdir(parents=True, exist_ok=False)
    node = None
    code = 1
    with (directory / 'commands.jsonl').open('x', encoding='utf-8') as log:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        try:
            node = Runner(args, log)
            node.event('scenario_started', scenario=args.scenario, count=args.count)
            print(f'Command log: {directory / "commands.jsonl"}', flush=True)
            node.run()
            code = 0
        except (ScenarioError, KeyboardInterrupt) as error:
            message = str(error) or 'Interrupted by operator'
            print(message, file=sys.stderr, flush=True)
            if node is not None:
                node.event('scenario_failed', reason=message)
                if node.started:
                    # Running this demo explicitly authorizes this documented
                    # error policy. An initial-state rejection sends no command.
                    try:
                        node.call('/swarm/hold', timeout=20.0, emergency=True)
                    except Exception as hold_error:
                        node.event('error_hold_failed', reason=str(hold_error))
                    try:
                        node.call('/swarm/evaluation/save', timeout=30.0, emergency=True)
                    except Exception as save_error:
                        node.event('error_save_failed', reason=str(save_error))
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
