import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.scan_msg_shown = False
        self.last_print_time = self.get_clock().now().seconds_nanoseconds()[0]

    def listener_callback(self, scan):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_print_time < 1:
            return  # Skip printing if less than 1 second has passed
        
        angle_min_deg = math.degrees(scan.angle_min)
        angle_increment_deg = math.degrees(scan.angle_increment)
        num_points = len(scan.ranges)

        def normalize_angle(angle):
            angle = (angle + 180) % 360 - 180
            return angle
        
        def angle_to_index(angle_robot_deg):
            angle_lidar_deg = (angle_robot_deg + 180) % 360
            if angle_lidar_deg > 180:
                angle_lidar_deg -= 360
            index = int(round((angle_lidar_deg - angle_min_deg) / angle_increment_deg))
            index = max(0, min(index, num_points - 1))
            return index

        index_0 = angle_to_index(0)
        index_neg90 = angle_to_index(-90)
        index_pos90 = angle_to_index(90)
        index_180 = angle_to_index(180)

        dist_0 = scan.ranges[index_0]
        dist_neg90 = scan.ranges[index_neg90]
        dist_pos90 = scan.ranges[index_pos90]
        dist_180 = scan.ranges[index_180]

        self.get_logger().info("---- LIDAR readings ----")
        self.get_logger().info(f"Distance at 0°: {dist_0:.2f} m" if dist_0 else "No valid reading at 0°")
        self.get_logger().info(f"Distance at -90°: {dist_neg90:.2f} m" if dist_neg90 else "No valid reading at -90°")
        self.get_logger().info(f"Distance at +90°: {dist_pos90:.2f} m" if dist_pos90 else "No valid reading at +90°")
        self.get_logger().info(f"Distance at 180º: {dist_180:.2f} m" if dist_180 else "No valid reading at 180°")

        custom_range = []
        for i, distance in enumerate(scan.ranges):
            if distance == float('inf') or distance == 0.0:
                continue
            custom_range.append((distance, i))

        if custom_range:
            closest_distance, element_index = min(custom_range)
            angle_closest_distance = normalize_angle(angle_min_deg + element_index * angle_increment_deg - 180)
            self.get_logger().info("---- LIDAR readings: Min distance ----")
            self.get_logger().info(f"Minimum distance: {closest_distance:.2f} m at angle {angle_closest_distance:.2f}°")

        self.last_print_time = current_time

def main(args=None):
    rclpy.init(args=args)
    lidar_test = LidarTest()
    rclpy.spin(lidar_test)
    lidar_test.destroy_node()
    rclpy.shutdown()