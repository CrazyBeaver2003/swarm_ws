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
        self.get_logger().info(f'Recording passive evaluation to {self.directory}')

    def subscribe(self, message_type, topic, callback, qos=qos_profile_sensor_data):
        self.subscriptions_owned.append(self.create_subscription(message_type,topic,callback,qos))

    def record(self,kind,stamp=None,**data):
        received = self.get_clock().now().nanoseconds*1e-9
        with self.lock:
            if self.closed:
                return
            if self.last_sim is not None and received < self.last_sim-1e-6:
                self.segment += 1
            self.last_sim = received
            row = clean(dict(kind=kind,segment=self.segment,t=received if stamp is None else stamp,
                             received_t=received,received_wall_elapsed_s=time.monotonic()-self.started_wall,**data))
            self.journal.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')

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
        self.pool.shutdown(wait=True)
        with self.lock:
            if not self.closed:
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
