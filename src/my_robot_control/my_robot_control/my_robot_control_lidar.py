#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RobotControlLidar(Node):
    def __init__(self):
        super().__init__('robot_control_lidar_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.moving = True

    def lidar_callback(self, scan):
        angle_min = math.degrees(scan.angle_min)
        angle_increment = math.degrees(scan.angle_increment)
        num_ranges = len(scan.ranges)

        def angle_to_index(angle_deg):
            angle_lidar = (angle_deg + 180) % 360
            if angle_lidar > 180:
                angle_lidar -= 360
            index = int(round((angle_lidar - angle_min) / angle_increment))
            return max(0, min(index, num_ranges - 1))

        front_index = angle_to_index(0)
        dist_front = scan.ranges[front_index]

        if self.moving and 0.01 < dist_front <= 0.3:
            # Found an obstacle at 30cm or less, stop!
            self.moving = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"Stopped: Wall detected {dist_front:.2f} m ahead at 0°"
            )

            # Also print closest
            valid_ranges = [(i, d) for i, d in enumerate(scan.ranges) if 0.01 < d < float('inf')]
            if valid_ranges:
                ind_closest, min_dist = min(valid_ranges, key=lambda x: x[1])
                angle_closest = angle_min + ind_closest * angle_increment - 180
                self.get_logger().info(
                    f"Closest object: {min_dist:.2f} m at angle {angle_closest:.2f}°"
                )
        elif self.moving:
            # No wall yet, keep moving forward
            twist = Twist()
            twist.linear.x = 0.2  # Safe speed
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()