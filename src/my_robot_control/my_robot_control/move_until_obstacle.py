import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class MoveUntilObstacle(Node):

    def __init__(self):
        super().__init__("move_until_obstacle")

        # --- PARAMETERS ---
        self.declare_parameter("stop_distance", 0.30)        # 30 cm
        self.declare_parameter("desired_linear_x", 0.20)     # m/s
        self.declare_parameter("desired_angular_z", 0.00)    # rad/s

        self.stop_distance = self.get_parameter("stop_distance").value
        self.target_vx = self.get_parameter("desired_linear_x").value
        self.target_wz = self.get_parameter("desired_angular_z").value

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10
        )

        # Timer for constant movement
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Movement state
        self.safe_to_move = True
        self.closest_distance_forward = None


    # ----------------------------------------------------------------------
    # MAIN MOVEMENT LOOP
    # ----------------------------------------------------------------------
    def timer_callback(self):
        msg = Twist()

        if self.safe_to_move:
            msg.linear.x = self.target_vx
            msg.angular.z = self.target_wz
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


    # ----------------------------------------------------------------------
    # PROCESS LIDAR: find distance in the direction of motion
    # ----------------------------------------------------------------------
def laser_callback(self, scan):
    angle_min_deg = scan.angle_min * 180.0 / math.pi
    angle_inc_deg = scan.angle_increment * 180.0 / math.pi

    forward_distances = []

    # Direction we are moving toward
    if self.target_vx > 0:        # forward
        target_center = 0.0
    elif self.target_vx < 0:      # backward
        target_center = 180.0
    else:                         # rotation-only movement
        target_center = 0.0

    window_deg = 10.0

    for i, d in enumerate(scan.ranges):
        angle = (angle_min_deg + i * angle_inc_deg) % 360

        if not math.isfinite(d):
            continue
        if d < scan.range_min or d > scan.range_max:
            continue

        # compute smallest angle difference
        diff = abs((angle - target_center + 180) % 360 - 180)

        if diff <= window_deg:
            forward_distances.append(d)

    # If no LIDAR data: assume safe to move
    if not forward_distances:
        self.safe_to_move = True
        return

    self.closest_distance_forward = min(forward_distances)

    if self.closest_distance_forward < self.stop_distance:
        if self.safe_to_move:
            self.get_logger().info(
                f"🛑 Stopping: obstacle at {self.closest_distance_forward:.2f} m"
            )
        self.safe_to_move = False
    else:
        self.safe_to_move = True


def main(args=None):
    rclpy.init(args=args)
    node = MoveUntilObstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()