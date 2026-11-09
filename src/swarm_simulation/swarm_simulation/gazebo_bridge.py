"""Gazebo ENU truth and clock; flight controllers must not consume truth topics."""

import math
from threading import Lock

import rclpy
from geometry_msgs.msg import PoseStamped
from gz.msgs10.clock_pb2 import Clock as GazeboClock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as RosClock


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        readonly = ParameterDescriptor(read_only=True)
        world = self.declare_parameter('world', 'default', readonly).value
        self.frame = self.declare_parameter('frame_id', 'swarm_map', readonly).value
        ids = self.declare_parameter('vehicle_ids', [1], readonly).value
        names = self.declare_parameter('model_names', ['rc_cessna_0'], readonly).value
        rate = self.declare_parameter('publish_rate_hz', 20.0, readonly).value
        if (not world or not self.frame or not ids or len(ids) != len(names)
                or len(set(ids)) != len(ids) or len(set(names)) != len(names)
                or any(i < 1 for i in ids) or any(not name for name in names)
                or not math.isfinite(rate) or rate <= 0):
            raise ValueError('Invalid world/frame, vehicle/model mapping, or rate')
        self._models = dict(zip(names, ids))
        self._interval_ns = max(1, round(1e9 / rate))
        self._clock_pub = self.create_publisher(RosClock, '/clock', 10)
        self._pose_pubs = {
            i: self.create_publisher(PoseStamped, f'/simulation/plane_{i}/pose', 10)
            for i in ids
        }
        self._lock = Lock()
        self._closed = False
        self._clock = None
        self._poses = {}  # At most one pending sample per configured vehicle.
        self._clock_seen = None
        self._pose_seen = None
        self._reset = False
        self._published = {}
        self._topics = [f'/world/{world}/clock', f'/world/{world}/pose/info']
        self._gz = GazeboNode()
        # A ROS-time timer would stop before /clock arrives or while paused.
        self._timer = self.create_timer(
            0.005, self._pump, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._world_service = f'/world/{world}/scene/info'

        self._connect_timer = self.create_timer(
            0.5,
            self._connect_to_world,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        self.get_logger().info(
            f'Gazebo bridge: waiting for world={world}, models={names}'
        )

    def _connect_to_world(self):
        if not self._gz.service_info(self._world_service):
            self.get_logger().info(
                f'Waiting for Gazebo service {self._world_service}',
                throttle_duration_sec=5.0,
            )
            return

        for msg_type, topic, callback in zip(
            [GazeboClock, Pose_V],
            self._topics,
            [self._on_clock, self._on_poses],
        ):
            if not self._gz.subscribe(msg_type, topic, callback):
                raise RuntimeError(f'Could not subscribe to {topic}')

        self._connect_timer.cancel()

        self.get_logger().info(
            'Gazebo world found; clock and pose subscriptions started'
        )
    def _on_clock(self, message):
        stamp = (message.sim.sec, message.sim.nsec)
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        with self._lock:
            if self._closed:
                return
            if self._clock_seen is not None and stamp_ns < self._clock_seen:
                self._reset = True
                self._poses.clear()
                self._pose_seen = None
            self._clock_seen = stamp_ns
            self._clock = stamp

    def _on_poses(self, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nsec)
        stamp_ns = stamp[0] * 1_000_000_000 + stamp[1]
        samples = {}
        for pose in message.pose:
            vehicle = self._models.get(pose.name)  # Exact top-level model names.
            if vehicle is None:
                continue
            p, q = pose.position, pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if all(math.isfinite(value) for value in values):
                samples[vehicle] = (stamp, stamp_ns, values)
        with self._lock:
            if self._closed:
                return
            # Only /clock defines a reset. Reject late and old-epoch packets.
            if self._clock_seen is None or stamp_ns > self._clock_seen + 1_000_000_000:
                return
            if self._pose_seen is not None and stamp_ns <= self._pose_seen:
                return
            self._pose_seen = stamp_ns
            self._poses.update(samples)

    def _pump(self):
        # Gazebo worker threads only store data; ROS publishes in this executor.
        with self._lock:
            clock, poses, reset = self._clock, self._poses, self._reset
            self._clock, self._poses, self._reset = None, {}, False
        if reset:
            self._published.clear()
        if clock is not None:
            message = RosClock()
            message.clock.sec, message.clock.nanosec = clock
            self._clock_pub.publish(message)
        for vehicle, (stamp, stamp_ns, values) in poses.items():
            previous = self._published.get(vehicle)
            if previous is not None and stamp_ns - previous < self._interval_ns:
                continue
            message = PoseStamped()
            message.header.stamp.sec, message.header.stamp.nanosec = stamp
            message.header.frame_id = self.frame
            p, q = message.pose.position, message.pose.orientation
            p.x, p.y, p.z, q.x, q.y, q.z, q.w = values
            self._pose_pubs[vehicle].publish(message)
            self._published[vehicle] = stamp_ns

    def destroy_node(self):
        with self._lock:
            self._closed = True
            self._clock = None
            self._poses.clear()
        self._timer.cancel()
        self._connect_timer.cancel()
        
        for topic in self._topics:
            self._gz.unsubscribe(topic)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GazeboBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
