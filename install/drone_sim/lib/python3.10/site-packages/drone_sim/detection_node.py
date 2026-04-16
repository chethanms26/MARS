import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class Detection(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/processed_image',
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 120, 70])
        upper = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)

                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)

                self.get_logger().info("🚨 Victim detected!")

                if self.current_pose:
                    self.get_logger().info(
                        f"x={self.current_pose.x:.2f}, y={self.current_pose.y:.2f}"
                    )

        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    rclpy.spin(node)
    rclpy.shutdown()
