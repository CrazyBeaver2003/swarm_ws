"""Passive ROS recorder. This node never publishes commands to aircraft."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from threading import Lock
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import MeshEstimate, RangeMeasurement, VehicleState

from .metrics import clean, export_report


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def xyz(value):
    return {'x':float(value.x),'y':float(value.y),'z':float(value.z)}


class EvaluationNode(Node):
    def __init__(self):
        super().__init__('swarm_evaluation')
        defaults = {
            'vehicle_ids':[1,2,3], 'frame_id':'swarm_map', 'output_directory':'evaluation_runs',
            'origin_alt_amsl_m':0.0, 'truth_max_gap_s':0.25, 'danger_distance_m':20.0,
            'airborne_min_height_m':10.0, 'goal_radius_m':30.0, 'goal_height_tolerance_m':10.0,
            'external_nav_state':23, 'run_name':'experiment', 'scenario':'unspecified',
        }
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        self.ids = tuple(self.get_parameter('vehicle_ids').value)
        self.frame = self.get_parameter('frame_id').value
        if (not self.get_parameter('use_sim_time').value or not self.ids or not self.frame
                or len(set(self.ids)) != len(self.ids) or any(not 1<=i<=255 for i in self.ids)):
            raise ValueError('Use simulation time, a frame, and unique vehicle IDs')
        for name in ('truth_max_gap_s','danger_distance_m','goal_radius_m','goal_height_tolerance_m'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value<=0:
                raise ValueError(f'Invalid {name}')
        name = str(self.get_parameter('run_name').value)
        if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
            raise ValueError('run_name must contain only letters, digits, underscore or hyphen')
        utc = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.directory = Path(self.get_parameter('output_directory').value).expanduser().resolve()/f'{utc}_{name}'
        self.directory.mkdir(parents=True,exist_ok=False)
        metadata = {key:self.get_parameter(key).value for key in defaults}
        metadata.update({'schema_version':1,'leader_id':self.ids[0],'started_utc':utc,
                         'provenance':'recorded_ros_topics','use_sim_time':True,
                         'truth_topic_template':'/simulation/plane_{id}/pose'})
        (self.directory/'metadata.json').write_text(json.dumps(clean(metadata),indent=2,allow_nan=False)+'\n')
        self.journal = (self.directory/'observations.jsonl').open('w',encoding='utf-8',buffering=1)
        self.lock = Lock()
        self.save_lock = Lock()
        self.pool = ThreadPoolExecutor(max_workers=1,thread_name_prefix='swarm-evaluation-export')
        self.last_sim = None
        self.segment = 0
        self.started_wall = time.monotonic()
        self.closed = False
        self.clock_changed_wall = None
        self.measurement = dict(phase='idle', measurement_id=0, segment=None,
                                start_sim_s=None, end_sim_s=None, duration_sim_s=0.0,
                                report_path=None, detail='Нажмите «Начать замер» после сборки строя.',
                                can_retry=False, summary=None)
        self.measurement_directory = None
        self.measurement_snapshot = None
        self.measurement_future = None
        self.subscriptions_owned = []
        for i in self.ids:
            self.subscribe(PoseStamped,f'/simulation/plane_{i}/pose',lambda msg,i=i:self.pose(msg,i))
            self.subscribe(MeshEstimate,f'/plane_{i}/mesh/estimate',lambda msg,i=i:self.estimate(msg,i))
            self.subscribe(VehicleState,f'/plane_{i}/state',lambda msg,i=i:self.state(msg,i))
            self.subscribe(RangeMeasurement,f'/plane_{i}/mesh/ranges',lambda msg,i=i:self.range(msg,i))
            if i != self.ids[0]:
                self.subscribe(PointStamped,f'/follower_{i}/target',lambda msg,i=i:self.target(msg,i))
        path_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(PathMessage,'/swarm/mission/path',self.route,path_qos)
        # Volatile readers are also compatible with latched publishers.
        self.subscribe(String,'/swarm/mission/status',lambda msg:self.json_message(msg,'mission'))
        self.subscribe(String,'/swarm/network/stats',lambda msg:self.json_message(msg,'network_stats'))
        self.subscribe(String,'/simulation/ranging_stats',lambda msg:self.json_message(msg,'ranging_stats'))
        self.service = self.create_service(Trigger,'/swarm/evaluation/save',self.save,
                                          callback_group=ReentrantCallbackGroup())
        self.measurement_services = [
            self.create_service(Trigger, '/swarm/evaluation/'+name, callback,
                                callback_group=ReentrantCallbackGroup())
            for name,callback in (('start',self.start_measurement),
                                  ('stop',self.stop_measurement),
                                  ('retry',self.retry_measurement))
        ]
        self.measurement_pub = self.create_publisher(
            String, '/swarm/evaluation/status', path_qos)
        self.measurement_timer = self.create_timer(
            0.5, self.publish_measurement_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_measurement_status()
        self.get_logger().info(f'Recording passive evaluation to {self.directory}')

    def subscribe(self, message_type, topic, callback, qos=qos_profile_sensor_data):
        self.subscriptions_owned.append(self.create_subscription(message_type,topic,callback,qos))

    def record(self,kind,stamp=None,**data):
        with self.lock:
            if self.closed:
                return
            received = self.get_clock().now().nanoseconds*1e-9
            self.observe_clock_locked(received)
            row = clean(dict(kind=kind,segment=self.segment,t=received if stamp is None else stamp,
                             received_t=received,received_wall_elapsed_s=time.monotonic()-self.started_wall,**data))
            self.journal.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')

    def observe_clock_locked(self, now):
        previous = self.last_sim
        if previous is None or abs(now-previous) > 1e-9:
            self.clock_changed_wall = time.monotonic()
        if previous is not None and now < previous-1e-6:
            self.segment += 1
            if self.measurement['phase'] == 'measuring':
                self.measurement.update(
                    phase='cancelled', end_sim_s=previous,
                    duration_sim_s=max(0.0, previous-self.measurement['start_sim_s']),
                    detail='Замер отменён: время симуляции сброшено. Начните новый замер.')
                try:
                    self.persist_measurement_locked()
                except OSError as error:
                    self.get_logger().error(f'Could not save cancelled measurement: {error}')
        self.last_sim = now

    def persist_measurement_locked(self):
        if self.measurement_directory is not None:
            path = self.measurement_directory/'measurement.json'
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(clean(self.measurement), ensure_ascii=False,
                                            indent=2, allow_nan=False)+'\n')
            temporary.replace(path)

    def start_measurement(self, request, response):
        response.success = False
        with self.lock:
            now = self.get_clock().now().nanoseconds*1e-9
            self.observe_clock_locked(now)
            if self.closed or self.measurement['phase'] in ('measuring','exporting'):
                response.message = 'Замер уже идёт или отчёт ещё рассчитывается.'
                return response
            if self.save_lock.locked():
                response.message = 'Дождитесь завершения сохранения полного полёта.'
                return response
            if (now <= 0 or self.clock_changed_wall is None
                    or time.monotonic()-self.clock_changed_wall > 3.0):
                response.message = 'Нет движущегося времени симуляции. Запустите или продолжите Gazebo.'
                return response
            identity = self.measurement['measurement_id']+1
            name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            directory = self.directory/'measurements'/f'{identity:04d}_{name}'
            try:
                directory.mkdir(parents=True, exist_ok=False)
                self.measurement_directory = directory
                self.measurement_snapshot = None
                self.measurement = dict(
                    phase='measuring', measurement_id=identity, segment=self.segment,
                    start_sim_s=now, end_sim_s=None, duration_sim_s=0.0,
                    report_path=None, can_retry=False, summary=None,
                    detail='Замер идёт. Завершите его после нужного участка полёта.')
                self.persist_measurement_locked()
            except OSError as error:
                self.measurement.update(phase='failed', detail=f'Не удалось начать замер: {error}')
                response.message = self.measurement['detail']
                return response
            response.success = True
            response.message = f'Замер #{identity} начат, время симуляции {now:.3f} с.'
        self.publish_measurement_status()
        return response

    def submit_measurement_locked(self):
        window = {key:self.measurement_snapshot[key]
                  for key in ('segment','start_sim_s','end_sim_s')}
        self.measurement.update(phase='exporting', can_retry=False,
                                detail='Замер остановлен. Рассчитываются метрики и графики…')
        self.persist_measurement_locked()
        self.measurement_future = self.pool.submit(
            export_report, self.directory, self.measurement_directory,
            self.measurement_snapshot['max_bytes'], True, window=window)

    def stop_measurement(self, request, response):
        response.success = False
        if not self.save_lock.acquire(blocking=False):
            response.message = 'Уже выполняется экспорт. Дождитесь его завершения.'
            return response
        submitted = False
        try:
            with self.lock:
                now = self.get_clock().now().nanoseconds*1e-9
                self.observe_clock_locked(now)
                if self.closed or self.measurement['phase'] != 'measuring':
                    response.message = 'Нет активного замера. Сначала нажмите «Начать замер».'
                    return response
                if now <= self.measurement['start_sim_s']:
                    response.message = 'Интервал пуст: дождитесь продвижения времени симуляции.'
                    return response
                self.journal.flush()
                self.measurement_snapshot = dict(
                    segment=self.measurement['segment'],
                    start_sim_s=self.measurement['start_sim_s'],
                    end_sim_s=now, max_bytes=self.journal.buffer.tell())
                self.measurement.update(end_sim_s=now,
                    duration_sim_s=now-self.measurement['start_sim_s'])
                (self.measurement_directory/'window.json').write_text(
                    json.dumps(self.measurement_snapshot, indent=2, allow_nan=False)+'\n')
                self.submit_measurement_locked()
                submitted = True
                response.success = True
                response.message = 'Интервал зафиксирован; расчёт начат. Дождитесь статуса «Готово».'
        except Exception as error:
            with self.lock:
                self.measurement.update(phase='failed', can_retry=self.measurement_snapshot is not None,
                                        detail=f'Не удалось запустить расчёт: {error}')
                response.message = self.measurement['detail']
        finally:
            if not submitted:
                self.save_lock.release()
        self.publish_measurement_status()
        return response

    def retry_measurement(self, request, response):
        response.success = False
        if not self.save_lock.acquire(blocking=False):
            response.message = 'Расчёт уже выполняется.'
            return response
        submitted = False
        try:
            with self.lock:
                if (self.closed or self.measurement['phase'] != 'failed'
                        or self.measurement_snapshot is None):
                    response.message = 'Нет сохранённого интервала для повторного расчёта.'
                    return response
                (self.measurement_directory/'window.json').write_text(
                    json.dumps(self.measurement_snapshot, indent=2, allow_nan=False)+'\n')
                self.submit_measurement_locked()
                submitted = True
                response.success = True
                response.message = 'Повторный расчёт того же интервала запущен.'
        except Exception as error:
            with self.lock:
                self.measurement.update(phase='failed', can_retry=True, detail=f'Ошибка расчёта: {error}')
                response.message = self.measurement['detail']
        finally:
            if not submitted:
                self.save_lock.release()
        self.publish_measurement_status()
        return response

    @staticmethod
    def report_summary(report):
        vehicles = {}
        for identity, values in report['vehicles'].items():
            localization, formation = values['localization_3d_m'], values['formation_relative_m']
            vehicles[identity] = dict(
                localization_rmse_m=localization['rmse'], localization_p95_m=localization['p95'],
                localization_count=localization['count'], formation_mean_m=formation['mean'],
                formation_max_m=formation['max'], formation_count=formation['count'],
                update_rate_hz=values['position_update_rate_hz'], invalid_estimates=values['invalid_estimates'])
        distances = [p['airborne_samples']['min'] for p in report['pairs'].values()
                     if p['airborne_samples']['count']]
        episodes = [p['airborne_observed_close_episodes'] for p in report['pairs'].values()
                    if p['airborne_observed_close_episodes'] is not None]
        missions = {identity:dict(
            phase=m['last_phase'], cross_track_rmse_m=m['cross_track_m']['rmse'],
            altitude_rmse_m=m['altitude_error_m']['rmse'], time_to_goal_s=m['reported_time_to_goal_s'])
            for identity,m in report['missions'].items() if m['last_phase'] != 'idle'}
        links = {}
        for short, key in (('rtt','ranging_counter_sessions'),('peer','network_counter_sessions')):
            values = list(report['links'][key].values())
            valid = [v for v in values if v.get('loss_fraction') is not None]
            generated = sum(v['generated'] for v in valid)
            delayed = [v for v in values if v.get('mean_delivery_delay_s') is not None
                       and isinstance(v.get('delivered'), int) and v['delivered'] > 0]
            delivered = sum(v['delivered'] for v in delayed)
            links[short] = dict(
                loss_fraction=sum(v['dropped'] for v in valid)/generated if generated else None,
                mean_delivery_delay_s=(sum(v['mean_delivery_delay_s']*v['delivered'] for v in delayed)
                                       /delivered if delivered else None))
        return dict(vehicles=vehicles, min_separation_m=min(distances) if distances else None,
                    close_episodes=sum(episodes) if episodes else None, missions=missions, links=links)

    def finish_measurement_locked(self):
        if self.measurement_future is None or not self.measurement_future.done():
            return
        try:
            result = self.measurement_future.result()
            self.measurement.update(
                phase='completed', can_retry=False,
                report_path=str(self.measurement_directory/'metrics.json'),
                summary=self.report_summary(result),
                detail='Готово. Метрики относятся к выбранному интервалу; полный журнал сохранён.')
        except Exception as error:
            self.measurement.update(phase='failed', can_retry=True, detail=f'Ошибка расчёта: {error}')
            self.get_logger().error(self.measurement['detail'])
        finally:
            self.measurement_future = None
            self.save_lock.release()
        try:
            self.persist_measurement_locked()
        except OSError as error:
            self.get_logger().error(f'Could not save measurement status: {error}')

    def publish_measurement_status(self):
        with self.lock:
            if self.closed:
                return
            now = self.get_clock().now().nanoseconds*1e-9
            self.observe_clock_locked(now)
            self.finish_measurement_locked()
            status = dict(self.measurement, export_busy=self.save_lock.locked())
            if status['phase'] == 'measuring':
                status['duration_sim_s'] = max(0.0, now-status['start_sim_s'])
        self.measurement_pub.publish(String(data=json.dumps(clean(status), ensure_ascii=False, allow_nan=False)))

    def valid(self,msg,vehicle_id=None):
        if msg.header.frame_id != self.frame:
            self.record('rejected',reason='frame mismatch',observed_frame=msg.header.frame_id)
            return False
        if vehicle_id is not None and hasattr(msg,'vehicle_id') and msg.vehicle_id != vehicle_id:
            self.record('rejected',reason='vehicle/topic mismatch',vehicle_id=vehicle_id)
            return False
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if stamp < 0 or stamp > now+0.05:
            self.record('rejected',reason='invalid/future source time',source_time=stamp)
            return False
        return True

    def pose(self,msg,i):
        if self.valid(msg):
            self.record('truth',stamp_seconds(msg.header.stamp),vehicle_id=i,**xyz(msg.pose.position))

    def estimate(self,msg,i):
        if self.valid(msg,i):
            self.record('estimate',stamp_seconds(msg.header.stamp),vehicle_id=i,**xyz(msg.position),
                        vx=msg.velocity.x,vy=msg.velocity.y,vz=msg.velocity.z,status=msg.status,
                        quality=msg.quality,measurement_age_s=msg.measurement_age_s,
                        reset_counter=msg.reset_counter,used_station_count=msg.used_station_count,
                        position_covariance=list(msg.position_covariance),velocity_covariance=list(msg.velocity_covariance))

    def state(self,msg,i):
        if self.valid(msg,i):
            self.record('state',stamp_seconds(msg.header.stamp),vehicle_id=i,**xyz(msg.position),
                        armed=msg.armed,failsafe=msg.failsafe,status_valid=msg.status_valid,
                        status_age_s=msg.status_age_s,nav_state=msg.nav_state,
                        nav_state_user_intention=msg.nav_state_user_intention,altitude_amsl_m=msg.altitude_amsl_m)

    def range(self,msg,i):
        if self.valid(msg,i):
            self.record('range',stamp_seconds(msg.header.stamp),vehicle_id=i,station_id=msg.station_id,
                        sequence=msg.sequence,valid=msg.valid,range_m=msg.range_m,
                        rtt_ns=msg.rtt_ns,turnaround_delay_ns=msg.turnaround_delay_ns,variance_m2=msg.variance_m2)

    def target(self,msg,i):
        if self.valid(msg):
            self.record('target',stamp_seconds(msg.header.stamp),vehicle_id=i,**xyz(msg.point))

    def route(self,msg):
        if self.valid(msg):
            points = [[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
            if len(points)>20000 or any(not all(math.isfinite(v) for v in p) for p in points):
                self.record('rejected',reason='invalid/oversized route')
                return
            self.record('route',stamp_seconds(msg.header.stamp),points=points)

    def json_message(self,msg,kind):
        if len(msg.data)>2_000_000:
            self.record('rejected',reason='oversized JSON message',source_kind=kind)
            return
        try:
            data = json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('Expected object')
            self.record(kind,data=data)
        except (ValueError,TypeError):
            self.record('rejected',reason='invalid JSON',source_kind=kind)

    def save(self,request,response):
        response.success = False
        if not self.save_lock.acquire(blocking=False):
            response.message = 'An export is already running'
            return response
        try:
            with self.lock:
                self.journal.flush()
                size = self.journal.buffer.tell()
            name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            output = self.directory/'exports'/name
            # Separate worker and executor thread: recording continues during export.
            future = self.pool.submit(export_report,self.directory,output,size,False)
            result = future.result()
            response.success = True
            response.message = f'{output}/metrics.json; observations={result["observations"]}'
        except Exception as error:
            response.message = f'Export failed: {error}'
            self.get_logger().error(response.message)
        finally:
            self.save_lock.release()
        return response

    def destroy_node(self):
        self.measurement_timer.cancel()
        self.pool.shutdown(wait=True)
        with self.lock:
            if not self.closed:
                self.finish_measurement_locked()
                if self.measurement['phase'] == 'measuring':
                    self.measurement.update(phase='cancelled', end_sim_s=self.last_sim,
                        duration_sim_s=max(0.0, (self.last_sim or 0)-self.measurement['start_sim_s']),
                        detail='Регистратор остановлен до завершения замера. Полный журнал сохранён.')
                    try:
                        self.persist_measurement_locked()
                    except OSError as error:
                        self.get_logger().error(f'Could not save cancelled measurement: {error}')
                self.closed = True
                self.journal.flush()
                self.journal.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = EvaluationNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
