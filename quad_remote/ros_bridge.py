"""Persistent ROS publishers, depth-one subscriptions and a separate executor."""
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv

from core import MODES


class RosBridge(Node):
    def __init__(self, latest):
        super().__init__('quad_web_bridge')
        self.latest = latest
        commands = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        # quad_rs / c620_quad publish JointState with RELIABLE; BEST_EFFORT won't connect.
        feedback = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', commands)
        self.pose_pubs = {mode: self.create_publisher(Empty, '/quad/teleop/' + mode, commands) for mode in MODES}
        self.create_subscription(String, '/quad/body_mode', lambda msg: latest.set_mode(msg.data), commands)
        self.create_subscription(JointState, '/quad/joint_states', lambda msg: latest.feedback('joints', msg), feedback)
        self.create_subscription(JointState, '/quad/wheel_states', lambda msg: latest.feedback('wheels', msg), feedback)
        self.create_subscription(Odometry, '/odom', lambda msg: latest.set_odom(msg), feedback)
        self._odom_reset = self.create_client(EmptySrv, '/quad/odom/reset')
        self._twist = Twist()
        self.create_timer(0.02, self.control_tick)
        self.create_timer(0.2, self.service_tick)
        self.create_timer(1.0, self.graph_tick)

    def graph_tick(self):
        # Only cmd_vel subscription matters for drive; pose topics may briefly read zero.
        self.latest.set_graph(self.velocity_pub.get_subscription_count() > 0)

    def service_tick(self):
        if not self.latest.consume_odom_reset():
            return
        if not self._odom_reset.service_is_ready():
            self.get_logger().warn('odom reset requested but /quad/odom/reset unavailable')
            return
        self._odom_reset.call_async(EmptySrv.Request())

    def control_tick(self):
        drive_out, pose = self.latest.tick()
        if drive_out is not None:
            self._twist.linear.x = float(drive_out.vx)
            self._twist.linear.y = float(drive_out.vy)
            self._twist.angular.z = float(drive_out.wz)
            self.velocity_pub.publish(self._twist)
        if pose is not None:
            self.pose_pubs[pose].publish(Empty())


class RosRuntime:
    def __init__(self, latest):
        rclpy.init(args=[])
        self.node = RosBridge(latest)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name='ros2', daemon=True)
        self.thread.start()

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        try:
            self.node.latest.stop()
            self.node.control_tick()
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
