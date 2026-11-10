"""Real GCS heartbeats on PX4 onboard links; QGC's separate link is untouched."""
import json
import time

from pymavlink import mavutil
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


class GcsLink(Node):
    def __init__(self):
        super().__init__('gcs_link')
        ids = self.declare_parameter('vehicle_ids', [1, 2, 3]).value
        base = self.declare_parameter('px4_port_base', 14580).value
        if not 3 <= len(ids) <= 5 or not 1024 <= base <= 65530:
            raise ValueError('Invalid GCS vehicle list or ports')
        self.links = {
            i: mavutil.mavlink_connection(
                f'udpout:127.0.0.1:{base + index}', source_system=254,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
            for index, i in enumerate(ids)
        }
        self.last_seen = {}
        self.publisher = self.create_publisher(String, '/swarm/gcs/status', 10)
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat, clock=clock)
        self.receive_timer = self.create_timer(0.05, self.receive, clock=clock)

    def receive(self):
        for i, link in self.links.items():
            for _ in range(100):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() == 'HEARTBEAT' and message.get_srcSystem() == i:
                    self.last_seen[i] = time.monotonic()

    def heartbeat(self):
        for link in self.links.values():
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
        now = time.monotonic()
        self.publisher.publish(String(data=json.dumps({
            str(i): {'connected': i in self.last_seen and now - self.last_seen[i] < 3.0}
            for i in self.links})))

    def destroy_node(self):
        for link in self.links.values():
            link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GcsLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
