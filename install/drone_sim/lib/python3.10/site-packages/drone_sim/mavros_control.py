import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DroneControl(Node):

    def __init__(self):
        super().__init__('mavros_control')

        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # 10 Hz publishing (required for OFFBOARD)
        self.timer = self.create_timer(0.1, self.move)

        self.waypoints = [
            (0.0, 0.0, 8.0),
            (5.0, 0.0, 8.0),
            (5.0, 5.0, 8.0),
            (0.0, 5.0, 8.0)
        ]

        self.index = 0
        self.counter = 0

    def move(self):
        msg = PoseStamped()

        # ✅ VERY IMPORTANT (fixes instability)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        x, y, z = self.waypoints[self.index]

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        self.pub.publish(msg)

        self.counter += 1

        # stay ~5 seconds per waypoint (50 * 0.1s)
        if self.counter > 50:
            self.counter = 0
            self.index = (self.index + 1) % len(self.waypoints)
            self.get_logger().info(f"➡️ Moving to waypoint {self.index}")



def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
