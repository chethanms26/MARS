import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np


class Detection(Node):

    def __init__(self):
        super().__init__('detection_node')

        self.bridge = CvBridge()

        # FIX: Gazebo camera plugin publishes with BEST_EFFORT reliability.
        # Using default RELIABLE QoS causes silent topic mismatch — no images received.
        cam_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            cam_qos          # FIX: was just "10" (RELIABLE) — now BEST_EFFORT
        )

        self.pub = self.create_publisher(
            Image,
            '/processed_image',
            10
        )

        # FIX: Also use BEST_EFFORT for pose — MAVROS publishes BEST_EFFORT
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            pose_qos          # FIX: was just "10" — now BEST_EFFORT
        )

        self.current_pose = None

        # Cooldown: log a detection once every N frames to avoid log spam
        self.detection_cooldown = 0
        self.COOLDOWN_FRAMES = 20

        self.get_logger().info('Detection node started — subscribing to /camera/image_raw')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red in HSV wraps around 0/180 boundary — must check BOTH ranges.
        # Range 1: 0–10  (lower red)
        # Range 2: 170–180 (upper red)
        lower_red1 = np.array([0,   120,  70])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 120,  70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask  = cv2.bitwise_or(mask1, mask2)

        # Morphological opening removes small noise blobs
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        victim_found = False
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:   # ignore tiny blobs
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(
                    frame, 'VICTIM', (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )
                victim_found = True

        if victim_found:
            if self.detection_cooldown <= 0:
                self.get_logger().info('*** VICTIM DETECTED! ***')
                if self.current_pose:
                    self.get_logger().info(
                        f'Location — x={self.current_pose.x:.2f}  '
                        f'y={self.current_pose.y:.2f}  '
                        f'z={self.current_pose.z:.2f}'
                    )
                    self.get_logger().info('Sending coordinates to rescue team...')
                else:
                    self.get_logger().warn(
                        'Victim detected but pose not yet available '
                        '(is /mavros/local_position/pose publishing?)'
                    )
                self.detection_cooldown = self.COOLDOWN_FRAMES
            else:
                self.detection_cooldown -= 1

        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()