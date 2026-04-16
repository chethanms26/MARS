import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


def generate_grid(xmin, xmax, ymin, ymax, spacing, altitude):
    """Generate a boustrophedon (lawn-mower) grid of waypoints."""
    waypoints = []
    row = 0
    y = ymin
    while y <= ymax:
        xs = list(range(xmin, xmax + 1, spacing))
        if row % 2 == 1:
            xs = list(reversed(xs))
        for x in xs:
            waypoints.append((float(x), float(y), float(altitude)))
        y += spacing
        row += 1
    return waypoints


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = None

        # Grid covers 30m x 30m at 8m altitude, 5m spacing
        self.waypoints = generate_grid(
            xmin=0, xmax=30,
            ymin=0, ymax=30,
            spacing=5,
            altitude=8
        )
        self.index = 0

        # PX4 OFFBOARD requires setpoints streaming BEFORE mode switch.
        # Stream for ~10 seconds (100 ticks at 0.1s) before attempting.
        self.prearm_counter = 0
        self.PREARM_COUNT = 100

        # FIX: Track whether we've already sent a request to avoid
        # rapid repeated calls that PX4 silently rejects.
        self.mode_requested = False
        self.arm_requested = False

        self.timer = self.create_timer(0.1, self.run)
        self.get_logger().info(
            f"Patrol node started — {len(self.waypoints)} waypoints generated"
        )

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def make_setpoint(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0
        return sp

    def run(self):
        if self.current_pose is None:
            return

        target = self.waypoints[self.index]

        # Always publish setpoint — required to keep OFFBOARD mode alive
        self.setpoint_pub.publish(self.make_setpoint(*target))
        self.prearm_counter += 1

        # Wait until we have been streaming setpoints long enough
        if self.prearm_counter < self.PREARM_COUNT:
            return

        # --- Step 1: Switch to OFFBOARD mode ---
        if self.current_state.mode != 'OFFBOARD':
            # FIX: Only send request ONCE — wait for state_cb to confirm
            if not self.mode_requested:
                req = SetMode.Request()
                req.custom_mode = 'OFFBOARD'
                self.mode_client.call_async(req)
                self.mode_requested = True
                self.get_logger().info('Requesting OFFBOARD mode...')
            return

        # Reset flag once OFFBOARD is confirmed
        self.mode_requested = False

        # --- Step 2: Arm the drone ---
        if not self.current_state.armed:
            # FIX: Only send arm request ONCE — wait for state_cb to confirm
            if not self.arm_requested:
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.arm_requested = True
                self.get_logger().info('Arming drone...')
            return

        # Reset flag once armed is confirmed
        self.arm_requested = False

        # --- Step 3: Navigate waypoints ---
        if self.is_reached(target):
            self.get_logger().info(
                f'Reached waypoint {self.index}: '
                f'({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})'
            )
            self.index = (self.index + 1) % len(self.waypoints)
            if self.index == 0:
                self.get_logger().info('Grid search complete — restarting.')

    def is_reached(self, target):
        dx = self.current_pose.position.x - target[0]
        dy = self.current_pose.position.y - target[1]
        dz = self.current_pose.position.z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) < 0.5


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()