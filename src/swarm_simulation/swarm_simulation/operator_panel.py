"""Local ENU operator panel. HTTP threads never call ROS APIs.

The transport pattern was reviewed against gpt_test/operator_panel.py; this
implementation uses the current service interfaces and explicit job results.
"""
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path as FilePath
import queue
import secrets
import threading
import time
from urllib.parse import urlsplit

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import MeshEstimate, VehicleState
from swarm_interfaces.srv import SetFormation, SetLeaderCommand, SetMissionGoal
import yaml


SERVICES = {
    'takeoff': ('/swarm/takeoff', Trigger, 1800.0),
    'activate': ('/swarm/activate', Trigger, 90.0),
    'hold': ('/swarm/hold', Trigger, 20.0),
    'formation': ('/swarm/set_formation', SetFormation, 20.0),
    'manual': ('/swarm/leader/set_command', SetLeaderCommand, 10.0),
    'mission': ('/swarm/mission/set_goal', SetMissionGoal, 20.0),
    'cancel': ('/swarm/mission/cancel', Trigger, 25.0),
    'save': ('/swarm/evaluation/save', Trigger, 30.0),
    'measurement_start': ('/swarm/evaluation/start', Trigger, 15.0),
    'measurement_stop': ('/swarm/evaluation/stop', Trigger, 15.0),
    'measurement_retry': ('/swarm/evaluation/retry', Trigger, 15.0),
}

MEASUREMENT_ACTIONS = frozenset(('measurement_start', 'measurement_stop', 'measurement_retry'))
MEASUREMENT_PHASES = frozenset(('idle', 'measuring', 'exporting', 'completed', 'failed', 'cancelled'))


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value


def number(value, name, lo, hi):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{name}: требуется число')
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{name}: допустимо {lo} … {hi}')
    return float(value)


def validate_command(payload, origin=0.0):
    if (not isinstance(payload, dict) or not isinstance(payload.get('action'), str)
            or payload['action'] not in SERVICES):
        raise ValueError('Неизвестная команда')
    action = payload['action']
    fields = {'formation': {'formation', 'spacing_m'},
              'manual': {'course_deg', 'altitude_amsl_m', 'equivalent_airspeed_m_s'},
              'mission': {'east_m', 'north_m', 'arrival_radius_m'}}.get(action, set())
    if set(payload) != fields | {'action'}:
        raise ValueError('Неверный набор полей команды')
    result = {'action': action}
    if action == 'formation':
        if payload['formation'] not in ('line', 'wedge'):
            raise ValueError('Формация: line или wedge')
        result.update(formation=payload['formation'],
                      spacing_m=number(payload['spacing_m'], 'Интервал, м', 45.0, 120.0))
    elif action == 'manual':
        course = math.radians(number(payload['course_deg'], 'Курс, градусы', 0.0, 360.0))
        result.update(course_rad=math.atan2(math.sin(course), math.cos(course)),
                      altitude_amsl_m=number(payload['altitude_amsl_m'], 'Высота AMSL, м',
                                             origin + 20.0, origin + 250.0),
                      equivalent_airspeed_m_s=number(payload['equivalent_airspeed_m_s'],
                                                     'EAS, м/с', 10.0, 19.0))
    elif action == 'mission':
        result.update(east_m=number(payload['east_m'], 'Восток, м', -10000.0, 10000.0),
                      north_m=number(payload['north_m'], 'Север, м', -10000.0, 10000.0),
                      arrival_radius_m=number(payload['arrival_radius_m'], 'Радиус цели, м',
                                              10.0, 100.0))
    return result


class BusyError(ValueError):
    pass


class PanelHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self, address, html, token, snapshot, submit):
        self.token = token
        self.html = html.replace(b'__PANEL_TOKEN__', token.encode('ascii'))
        self.snapshot = snapshot
        self.submit = submit
        super().__init__(address, PanelRequestHandler)
        port = self.server_address[1]
        self.allowed_hosts = {f'127.0.0.1:{port}', f'localhost:{port}'}


class PanelRequestHandler(BaseHTTPRequestHandler):
    server_version = 'SwarmPanel/2.0'

    def reply(self, status, body, content_type='application/json; charset=utf-8'):
        if not isinstance(body, bytes):
            body = json.dumps(finite_json(body), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; "
                         "style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def valid_host(self):
        if self.headers.get('Host', '') not in self.server.allowed_hosts:
            self.reply(403, {'error': 'Используйте локальный адрес панели'})
            return False
        return True

    def do_GET(self):
        if not self.valid_host():
            return
        path = urlsplit(self.path).path
        if path in ('/', '/panel.html'):
            self.reply(200, self.server.html, 'text/html; charset=utf-8')
        elif path == '/api/state':
            self.reply(200, self.server.snapshot())
        elif path == '/favicon.ico':
            self.reply(204, b'')
        else:
            self.reply(404, {'error': 'Страница не найдена'})

    def do_POST(self):
        if not self.valid_host():
            return
        if urlsplit(self.path).path != '/api/command':
            self.reply(404, {'error': 'Команда не найдена'})
            return
        host = self.headers.get('Host', '')
        if self.headers.get('Origin') not in (None, f'http://{host}'):
            self.reply(403, {'error': 'Команда должна поступать со страницы панели'})
            return
        if not secrets.compare_digest(self.headers.get('X-Panel-Token', ''), self.server.token):
            self.reply(403, {'error': 'Обновите страницу панели'})
            return
        if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
            self.reply(415, {'error': 'Требуется application/json'})
            return
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 1 <= size <= 8192:
                raise ValueError('Недопустимый размер запроса')
            self.connection.settimeout(3.0)
            payload = json.loads(self.rfile.read(size).decode('utf-8'))
            job_id = self.server.submit(payload)
        except BusyError as error:
            self.reply(409, {'error': str(error)})
            return
        except (ValueError, UnicodeError, TimeoutError) as error:
            self.reply(400, {'error': str(error)})
            return
        except queue.Full:
            self.reply(429, {'error': 'Очередь заполнена'})
            return
        self.reply(202, {'job_id': job_id, 'accepted': True,
                         'message': 'Запрос поставлен в очередь; результат появится в журнале'})

    def log_message(self, *_args):
        pass


class OperatorPanel(Node):
    def __init__(self):
        super().__init__('operator_panel')
        defaults = {'vehicle_ids': [1, 2, 3], 'frame_id': 'swarm_map',
                    'origin_alt_amsl_m': 0.0, 'stations_file': '',
                    'host': '127.0.0.1', 'port': 8080}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.ids = tuple(self.get_parameter('vehicle_ids').value)
        self.frame = self.get_parameter('frame_id').value
        self.origin = float(self.get_parameter('origin_alt_amsl_m').value)
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        if (not 3 <= len(self.ids) <= 5 or len(set(self.ids)) != len(self.ids)
                or not all(isinstance(i, int) and 1 <= i <= 255 for i in self.ids)
                or not self.frame or not math.isfinite(self.origin)):
            raise ValueError('Need 3..5 unique plane IDs, a frame and finite origin')
        if host != '127.0.0.1' or not isinstance(port, int) or not 1024 <= port <= 65535:
            raise ValueError('Use host=127.0.0.1 and port=1024..65535')
        self.lock = threading.Lock()
        self.queue = queue.PriorityQueue(maxsize=32)
        self.jobs = {}
        self.active_actions = set()
        self.next_job = 0
        self.pending = {}
        self.vehicles = {i: {} for i in self.ids}
        self.targets = {}
        self.path = []
        self.mission = {'phase': 'waiting'}
        self.network = {}
        self.measurement = {'phase': 'waiting', 'stale': True}
        self.measurement_received = None
        self.stations = self.load_stations(self.get_parameter('stations_file').value)
        self.snapshot_data = {'vehicles': [], 'jobs': [], 'mission': {'phase': 'waiting'},
                              'measurement': dict(self.measurement),
                              'stations': self.stations, 'origin_alt_amsl_m': self.origin}
        self.service_clients = {action: self.create_client(service, name)
                        for action, (name, service, _) in SERVICES.items()}
        self.subscriptions_ = []
        for i in self.ids:
            self.subscriptions_.append(self.create_subscription(
                MeshEstimate, f'/plane_{i}/mesh/estimate',
                lambda msg, i=i: self.receive_mesh(i, msg), qos_profile_sensor_data))
            self.subscriptions_.append(self.create_subscription(
                VehicleState, f'/plane_{i}/state',
                lambda msg, i=i: self.receive_state(i, msg), qos_profile_sensor_data))
            if i != self.ids[0]:
                self.subscriptions_.append(self.create_subscription(
                    PointStamped, f'/follower_{i}/target',
                    lambda msg, i=i: self.receive_target(i, msg), 1))
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_.append(self.create_subscription(
            Path, '/swarm/mission/path', self.receive_path, path_qos))
        self.subscriptions_.append(self.create_subscription(
            String, '/swarm/mission/status', self.receive_mission, 10))
        self.subscriptions_.append(self.create_subscription(
            String, '/swarm/network/stats', self.receive_network, 10))
        self.subscriptions_.append(self.create_subscription(
            String, '/swarm/evaluation/status', self.receive_measurement, path_qos))
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady)
        html_path = FilePath(__file__).with_name('panel.html')
        if not html_path.exists():
            from ament_index_python.packages import get_package_share_directory
            html_path = FilePath(get_package_share_directory('swarm_simulation')) / 'panel.html'
        self.http = PanelHTTPServer((host, port), html_path.read_bytes(),
                                    secrets.token_hex(24), self.snapshot, self.submit)
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Operator panel: http://{host}:{port}')

    def load_stations(self, filename):
        if not filename:
            from ament_index_python.packages import get_package_share_directory
            filename = str(FilePath(get_package_share_directory('swarm_bringup')) / 'config/stations.yaml')
        raw = yaml.safe_load(FilePath(filename).read_text())
        params = raw.get('/**', raw).get('ros__parameters', raw)
        stations = []
        for i in params.get('station_ids', []):
            point = params.get(f'station_{i}_position_enu_m')
            if not isinstance(point, list) or len(point) != 3:
                raise ValueError(f'Invalid station {i}')
            point = [number(v, 'Координата станции', -1e6, 1e6) for v in point]
            stations.append({'id': i, 'position': point})
        if len(stations) != 3:
            raise ValueError('Exactly three station coordinates required')
        return stations

    def stamp(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def receive_mesh(self, i, msg):
        if msg.vehicle_id != i or msg.header.frame_id != self.frame:
            return
        self.vehicles[i]['mesh'] = {
            'position': [msg.position.x, msg.position.y, msg.position.z],
            'velocity': [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            'quality': msg.quality, 'mesh_status': int(msg.status),
            'measurement_age_s': msg.measurement_age_s,
            'received': time.monotonic(), 'stamp': self.stamp(msg.header),
        }

    def receive_state(self, i, msg):
        if msg.vehicle_id != i or msg.header.frame_id != self.frame:
            return
        self.vehicles[i]['state'] = {
            'armed': msg.armed, 'failsafe': msg.failsafe,
            'nav_state': int(msg.nav_state), 'nav_state_user_intention': int(msg.nav_state_user_intention),
            'status_valid': msg.status_valid, 'status_age_s': msg.status_age_s,
            'altitude_amsl_m': msg.altitude_amsl_m if msg.altitude_valid else None,
            'received': time.monotonic(), 'stamp': self.stamp(msg.header),
        }

    def receive_target(self, i, msg):
        if msg.header.frame_id == self.frame:
            self.targets[i] = {'position': [msg.point.x, msg.point.y, msg.point.z],
                               'received': time.monotonic()}

    def receive_path(self, msg):
        if msg.header.frame_id == self.frame:
            self.path = [[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                         for p in msg.poses[:10000]]

    def receive_mission(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.mission = value
        except (ValueError, TypeError):
            pass

    def receive_network(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.network = value
        except (ValueError, TypeError):
            pass

    def receive_measurement(self, msg):
        # A new browser connection receives the recorder's current state through
        # this latched topic. Local HTTP acceptance is never a measurement result.
        if len(msg.data) > 262144:
            return
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or value.get('phase') not in MEASUREMENT_PHASES:
                return
            self.measurement = value
            self.measurement_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def snapshot(self):
        with self.lock:
            result = deepcopy(self.snapshot_data)
            measurement = result.setdefault('measurement', {})
            age = (None if self.measurement_received is None
                   else max(0.0, time.monotonic() - self.measurement_received))
            measurement.update(receipt_age_s=age, stale=age is None or age > 3.0)
            return result

    def submit(self, payload):
        command = validate_command(payload, self.origin)
        action = command['action']
        with self.lock:
            if action in self.active_actions:
                raise BusyError('Предыдущий запрос этой команды ещё выполняется')
            if action in MEASUREMENT_ACTIONS:
                if (MEASUREMENT_ACTIONS.intersection(self.active_actions)
                        or 'save' in self.active_actions):
                    raise BusyError('Предыдущий запрос замера ещё выполняется')
                measurement = self.snapshot_data.get('measurement', {})
                if (measurement.get('stale', True) or self.measurement_received is None
                        or time.monotonic() - self.measurement_received > 3.0):
                    raise BusyError('Нет свежего состояния регистратора метрик')
                if measurement.get('export_busy', False):
                    raise BusyError('Регистратор ещё рассчитывает предыдущий отчёт')
                phase = measurement.get('phase')
                if action == 'measurement_start' and phase not in ('idle', 'completed', 'failed', 'cancelled'):
                    raise BusyError('Предыдущий замер ещё не завершён')
                if action == 'measurement_stop' and phase != 'measuring':
                    raise BusyError('Нет активного замера для завершения')
                if action == 'measurement_retry' and not (phase == 'failed' and measurement.get('can_retry') is True):
                    raise BusyError('Нет сохранённого интервала для повторного расчёта')
            self.next_job += 1
            job_id = self.next_job
            job = {'id': job_id, 'action': action, 'state': 'queued',
                   'message': 'Ожидание обработки', 'created': time.monotonic()}
            self.queue.put_nowait((0 if action in ('hold', 'cancel') else 10, job_id, command))
            self.jobs[job_id] = job
            self.active_actions.add(action)
            # Retain active jobs and the last 64 completed responses.
            finished = [key for key, value in self.jobs.items()
                        if value['state'] in ('succeeded', 'failed', 'timed_out')]
            for key in sorted(finished)[:-64]:
                self.jobs.pop(key, None)
        return job_id

    def finish_job(self, job_id, state, message):
        with self.lock:
            job = self.jobs[job_id]
            job.update(state=state, message=message, finished=time.monotonic())
            self.active_actions.discard(job['action'])
        self.pending.pop(job_id, None)

    def tick(self):
        now = time.monotonic()
        for _ in range(8):
            try:
                _, job_id, command = self.queue.get_nowait()
            except queue.Empty:
                break
            self.pending[job_id] = {'command': command, 'future': None, 'waiting_since': now}
        for job_id, task in list(self.pending.items()):
            action = task['command']['action']
            client = self.service_clients[action]
            if task['future'] is None:
                if not client.service_is_ready():
                    with self.lock:
                        self.jobs[job_id].update(state='waiting_service', message=f'Ожидание {SERVICES[action][0]}')
                    if now - task['waiting_since'] > 5.0:
                        self.finish_job(job_id, 'failed', f'Сервис недоступен: {SERVICES[action][0]}; ничего не отправлено')
                    continue
                request = SERVICES[action][1].Request()
                for name, value in task['command'].items():
                    if name != 'action':
                        setattr(request, name, value)
                try:
                    task['future'] = client.call_async(request)
                    task['sent'] = now
                    with self.lock:
                        self.jobs[job_id].update(state='running', message='Запрос отправлен, ожидание ответа')
                except Exception as error:
                    self.finish_job(job_id, 'failed', str(error))
                    continue
            future = task['future']
            if future.done():
                try:
                    result = future.result()
                    message = result.message
                    if action == 'mission' and result.success:
                        message = 'Цель принята. Завершение полёта смотрите в состоянии миссии. ' + message
                    elif action == 'measurement_start' and result.success:
                        message = 'Начало замера принято. Состояние и время показаны в разделе измерений. ' + message
                    elif action in ('measurement_stop', 'measurement_retry') and result.success:
                        message = 'Завершение замера принято. Дождитесь состояния «Отчёт готов». ' + message
                    self.finish_job(job_id, 'succeeded' if result.success else 'failed', message)
                except Exception as error:
                    self.finish_job(job_id, 'failed', str(error))
            elif now - task['sent'] > SERVICES[action][2]:
                future.cancel()  # Cancels local waiting only; never resend implicitly.
                self.finish_job(job_id, 'timed_out',
                                'Ответ не получен. Удалённая команда могла выполниться; проверьте режимы. Повтор не отправлен.')
        sim = self.get_clock().now().nanoseconds * 1e-9
        vehicles = []
        for i, entry in self.vehicles.items():
            item = {'id': i}
            for kind in ('mesh', 'state'):
                sample = entry.get(kind)
                if sample is not None:
                    sample = dict(sample)
                    sample['receipt_age_s'] = max(0.0, now - sample.pop('received'))
                    sample['header_age_s'] = sim - sample.pop('stamp')
                    sample['stale'] = sample['receipt_age_s'] > 1.0 or not -0.02 <= sample['header_age_s'] <= 0.5
                    if kind == 'state':
                        sample['stale'] = (sample['stale'] or not sample['status_valid']
                            or not math.isfinite(sample['status_age_s'])
                            or sample['status_age_s'] < 0.0
                            or sample['status_age_s'] + sample['receipt_age_s'] > 2.0)
                    item[kind] = sample
            vehicles.append(item)
        targets = {str(i): data['position'] for i, data in self.targets.items() if now - data['received'] <= 1.0}
        measurement = dict(self.measurement)
        measurement['receipt_age_s'] = (None if self.measurement_received is None
                                         else max(0.0, now - self.measurement_received))
        measurement['stale'] = (self.measurement_received is None
                                  or now - self.measurement_received > 3.0)
        with self.lock:
            self.snapshot_data = finite_json({
                'vehicles': vehicles, 'stations': self.stations, 'targets': targets,
                'path': self.path, 'mission': self.mission, 'network': self.network,
                'measurement': measurement,
                'jobs': list(sorted(self.jobs.values(), key=lambda j: j['id'], reverse=True))[:12],
                'sim_time_s': sim, 'origin_alt_amsl_m': self.origin, 'frame_id': self.frame,
            })

    def destroy_node(self):
        if hasattr(self, 'http'):
            self.http.shutdown()
            self.http.server_close()
            self.thread.join(timeout=1.0)
        for task in self.pending.values():
            if task['future'] is not None:
                task['future'].cancel()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OperatorPanel()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
