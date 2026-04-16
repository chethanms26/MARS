import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)

        # Publisher
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = None

        self.waypoints = [(0, 0, 5), (5, 0, 5), (5, 5, 5), (0, 5, 5)]
        self.index = 0

        self.timer = self.create_timer(0.1, self.run)

        self.get_logger().info("🚁 Patrol node started")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def run(self):

        if self.current_pose is None:
            return

        target = self.waypoints[self.index]

        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.pose.position.x = float(target[0])
        setpoint.pose.position.y = float(target[1])
        setpoint.pose.position.z = float(target[2])
        setpoint.pose.orientation.w = 1.0

        # 🔥 MUST publish continuously
        self.setpoint_pub.publish(setpoint)

        # 🔥 Switch to OFFBOARD
        if self.current_state.mode != "OFFBOARD":
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            self.mode_client.call_async(req)
            self.get_logger().info("Switching to OFFBOARD")

        # 🔥 Arm drone
        if not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.get_logger().info("Arming drone")

        # Move to next waypoint
        if self.is_reached(target):
            self.index = (self.index + 1) % len(self.waypoints)
            self.get_logger().info(f"➡ Moving to waypoint {self.index}")

    def is_reached(self, target):
        dx = self.current_pose.position.x - target[0]
        dy = self.current_pose.position.y - target[1]
        dz = self.current_pose.position.z - target[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz) < 0.5


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()