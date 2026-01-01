import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.15)  # Reduced from 0.25 for better detection
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('side_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('time_to_stop', 30.0)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.forwardSpeed = float(self.get_parameter('forward_speed').value)
        self.sideSpeed = self.get_parameter('side_speed').value
        self.rotationSpeed = self.get_parameter('rotation_speed').value
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)

        # Wall alignment parameters
        self.DESIRED_WALL_DIST = 0.25  # Target distance to wall (meters)
        self.ALIGNMENT_TOLERANCE = 0.02  # Tolerance for parallel alignment (meters)
        self.FRONTAL_PRIORITY = 0.1  # Bonus distance for frontal walls (prioritize front walls)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        # Wall alignment state machine
        self._aligning_to_wall = False
        self._target_wall_side = None  # "RIGHT", "LEFT", "FRONT"
        self._target_angle_error = 0.0

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower with wall alignment - detects walls and aligns before moving."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True
        self.cmd = Twist()

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    #--------------------------------------------------------------------
    def find_closest_wall(self, min_front, min_left, min_right, min_fr_left, min_fr_right):
        """
        Detects which wall is closest, prioritizing frontal walls in case of ties.
        
        Returns:
            tuple: (closest_distance, wall_side, left_dist, right_dist)
            wall_side: "FRONT", "LEFT", "RIGHT", or None
            left_dist, right_dist: distances for angle calculation
        """
        walls = []

        # Frontal wall detection (has priority bonus)
        if math.isfinite(min_front):
            walls.append((min_front - self.FRONTAL_PRIORITY, "FRONT", min_front, None))

        # Left wall detection
        if math.isfinite(min_left):
            walls.append((min_left, "LEFT", min_left, min_fr_left))

        # Right wall detection
        if math.isfinite(min_right):
            walls.append((min_right, "RIGHT", min_right, min_fr_right))

        if not walls:
            return float('inf'), None, None, None

        # Sort by distance (lower first), frontal priority handled by negative bonus
        walls.sort(key=lambda x: x[0])
        closest_dist, wall_side, primary_dist, secondary_dist = walls[0]

        return closest_dist, wall_side, primary_dist, secondary_dist

    #--------------------------------------------------------------------
    def calculate_wall_angle_error(self, primary_dist, secondary_dist, wall_side):
        """
        Calculate angle error for wall alignment.
        Positive error = robot angled away from wall, needs correction toward wall.
        
        For RIGHT wall: primary=RIGHT distance, secondary=BACK_RIGHT distance
        For LEFT wall: primary=LEFT distance, secondary=BACK_LEFT distance
        For FRONT wall: no angle error (wall is perpendicular)
        """
        if wall_side == "FRONT":
            return 0.0

        if not math.isfinite(secondary_dist):
            return 0.0

        # error_angle = primary_dist - secondary_dist
        # Positive = robot angled away from wall
        return primary_dist - secondary_dist

    #--------------------------------------------------------------------
    def is_wall_aligned(self, angle_error):
        """Check if robot is parallel to wall within tolerance."""
        return abs(angle_error) < self.ALIGNMENT_TOLERANCE

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """
        Main control logic with wall detection and alignment.
        State machine:
        1. Emergency avoidance (collision imminent)
        2. Wall detection & alignment
        3. Wall following (main behavior)
        4. Wall search
        """
        if self._shutting_down:
            return

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        # Initialize sector arrays
        FRONT      = []
        FR_RIGHT   = []
        RIGHT      = []
        BACK_RIGHT = []
        BACK       = []
        BACK_LEFT  = []
        LEFT       = []
        FR_LEFT    = []

        # Step 1: Classify LIDAR rays by angular sector
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -math.radians(20) <= ang <= math.radians(20):
                FRONT.append(d)
            elif -math.radians(60) <= ang < -math.radians(20):
                FR_RIGHT.append(d)
            elif -math.radians(120) <= ang < -math.radians(60):
                RIGHT.append(d)
            elif -math.radians(160) <= ang < -math.radians(120):
                BACK_RIGHT.append(d)
            elif math.radians(20) < ang <= math.radians(60):
                FR_LEFT.append(d)
            elif math.radians(60) < ang <= math.radians(120):
                LEFT.append(d)
            elif math.radians(120) < ang <= math.radians(160):
                BACK_LEFT.append(d)
            else:
                BACK.append(d)

        # Step 2: Extract minimum distances per sector
        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_fr_left    = min(FR_LEFT)    if FR_LEFT    else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')
        min_back_left  = min(BACK_LEFT)  if BACK_LEFT  else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')

        # ========================================================================
        # STATE 1: EMERGENCY COLLISION AVOIDANCE
        # Triggered when ANY obstacle is extremely close
        # ========================================================================
        closest_dist = min(min_front, min_left, min_right, min_back, 
                        min_fr_left, min_fr_right, min_back_left, min_back_right)

        twist = Twist()
        if closest_dist < self.base_distance * 0.75:  # Even tighter emergency threshold
            directions = {
                "FRONT":      min_front,
                "LEFT":       min_left,
                "RIGHT":      min_right,
                "BACK":       min_back,
            }
            best_zone = max(directions, key=directions.get)

            if best_zone == "FRONT" or best_zone == "LEFT":
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                action = "EMERGENCY: Turn LEFT"
            elif best_zone == "RIGHT":
                twist.linear.x = 0.0
                twist.angular.z = -0.5
                action = "EMERGENCY: Turn RIGHT"
            else:
                twist.linear.x = -self.forwardSpeed
                twist.angular.z = 0.0
                action = "EMERGENCY: Move BACK"

            self._aligning_to_wall = False
            self.cmd = twist
            self._state_action = action
            return

        # ========================================================================
        # STATE 2: WALL DETECTION & ALIGNMENT
        # Triggered when a wall is detected at reduced distance
        # Robot stops, aligns parallel to wall, then proceeds
        # ========================================================================
        closest_wall_dist, wall_side, primary_dist, secondary_dist = self.find_closest_wall(
            min_front, min_left, min_right, min_fr_left, min_fr_right
        )

        if closest_wall_dist < self.base_distance and not self._aligning_to_wall:
            # Wall detected! Enter alignment mode
            self._aligning_to_wall = True
            self._target_wall_side = wall_side
            angle_error = self.calculate_wall_angle_error(primary_dist, secondary_dist, wall_side)
            self._target_angle_error = angle_error

            action = f"Wall detected ({wall_side}) at {closest_wall_dist:.3f}m → Starting alignment"
            self.cmd = Twist()  # Stop all motion
            self._state_action = action
            return

        # ========================================================================
        # STATE 2B: WALL ALIGNMENT IN PROGRESS
        # Rotate until robot is parallel to detected wall
        # ========================================================================
        if self._aligning_to_wall:
            angle_error = self.calculate_wall_angle_error(primary_dist, secondary_dist, self._target_wall_side)

            # Check if alignment is complete
            if self.is_wall_aligned(angle_error):
                self._aligning_to_wall = False
                action = f"Wall aligned! ({self._target_wall_side}) - Ready to move"
                self.cmd = Twist()
                self._state_action = action
                return

            # Rotate toward wall with proportional control
            # For RIGHT wall: negative rotation (clockwise)
            # For LEFT wall: positive rotation (counterclockwise)
            # For FRONT wall: already aligned (perpendicular)
            rotation_direction = -1.0 if self._target_wall_side == "RIGHT" else 1.0

            if self._target_wall_side == "FRONT":
                twist.angular.z = 0.0
            else:
                # Proportional rotation: stronger rotation for larger angle errors
                twist.angular.z = rotation_direction * min(0.5, abs(angle_error) * 2.0)

            twist.linear.x = 0.0
            action = f"Aligning to {self._target_wall_side} wall | angle_err={angle_error:.3f}"
            self.cmd = twist
            self._state_action = action
            return

        # ========================================================================
        # STATE 3: FRONTAL OBSTACLE (normal operation)
        # ========================================================================
        if min_front < self.base_distance * 2:
            twist.linear.x = 0.0
            twist.angular.z = 0.4
            action = "Obstacle FRONT → Turn LEFT"
            self.cmd = twist
            self._state_action = action
            return

        # ========================================================================
        # STATE 4: LEFT SIDE CORRECTION
        # ========================================================================
        if min_left < self.base_distance:
            twist.linear.x = self.forwardSpeed
            twist.angular.z = -0.4
            action = "Obstacle LEFT → Correct RIGHT"
            self.cmd = twist
            self._state_action = action
            return

        # ========================================================================
        # STATE 5: WALL FOLLOWING (Primary Behavior)
        # Tracks a wall on the right side at constant desired distance
        # ========================================================================
        if math.isfinite(min_right):
            error_dist = min_right - self.DESIRED_WALL_DIST

            if math.isfinite(min_back_right):
                error_angle = min_right - min_back_right
            else:
                error_angle = 0.0

            twist.linear.x = self.forwardSpeed
            twist.angular.z = -1.2 * error_dist - 1.0 * error_angle

            action = f"Following RIGHT wall | dist_err={error_dist:.2f} | angle_err={error_angle:.2f}"
            self.cmd = twist
            self._state_action = action
            return

        # ========================================================================
        # STATE 6: WALL SEARCH (Lost the wall)
        # ========================================================================
        twist.linear.x = self.forwardSpeed
        twist.angular.z = -0.5
        action = "Searching RIGHT wall"
        self.cmd = twist
        self._state_action = action

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()