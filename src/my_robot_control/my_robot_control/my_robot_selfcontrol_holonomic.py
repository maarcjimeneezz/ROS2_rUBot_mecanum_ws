import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Configurable parameters
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('side_speed', 0.15)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('time_to_stop', 5.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._sideSpeed = self.get_parameter('side_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.linear.y = 0.0
        self._msg.angular.z = 0.0

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._last_info_time = self.start_time
        self._last_speed_time = self.start_time

    def timer_callback(self):
        if self._shutting_down:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = now_sec - self.start_time

        self._cmdVel.publish(self._msg)

        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(
                f"Vx: {self._msg.linear.x:.2f} | Vy: {self._msg.linear.y:.2f} | w: {self._msg.angular.z:.2f} | t: {elapsed_time:.1f}s"
            )
            self._last_speed_time = now_sec

        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info("Robot stopped")
            rclpy.try_shutdown()

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min_deg = scan.angle_min * 180.0 / math.pi
        angle_inc_deg = scan.angle_increment * 180.0 / math.pi

        # Inicializamos distancias por zonas
        zones = {
            "FRONT": [],
            "FRONT_LEFT": [],
            "FRONT_RIGHT": [],
            "LEFT": [],
            "RIGHT": [],
            "BACK_LEFT": [],
            "BACK_RIGHT": [],
            "BACK": []
        }

        # Clasificación de todos los puntos
        for i, dist in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_inc_deg
            if angle_deg > 180:
                angle_deg -= 360

            if not math.isfinite(dist) or dist <= 0.0:
                continue
            if dist < scan.range_min or dist > scan.range_max:
                continue

            # Clasificamos ángulo en zonas
            if -30 <= angle_deg <= 30:
                zones["FRONT"].append(dist)
            elif -60 < angle_deg <= -30:
                zones["FRONT_RIGHT"].append(dist)
            elif -120 < angle_deg <= -60:
                zones["RIGHT"].append(dist)
            elif -150 < angle_deg <= -120:
                zones["BACK_RIGHT"].append(dist)
            elif 30 < angle_deg <= 60:
                zones["FRONT_LEFT"].append(dist)
            elif 60 < angle_deg <= 120:
                zones["LEFT"].append(dist)
            elif 120 < angle_deg <= 150:
                zones["BACK_LEFT"].append(dist)
            else:
                zones["BACK"].append(dist)

        # Tomamos la distancia mínima en cada zona
        zone_min = {
            z: min(dist_list) if dist_list else 999.0   # Si no hay lecturas, asumimos que está despejado
            for z, dist_list in zones.items()
        }

        # Si el frente está despejado → avanzamos
        if zone_min["FRONT"] > self._distanceLimit:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.linear.y = 0.0
            self._msg.angular.z = 0.0
            return

        # ------------------------------------------------------
        #   ELECCIÓN: mover hacia la zona MÁS DESPEJADA
        # ------------------------------------------------------
        best_zone = max(zone_min, key=zone_min.get)  # zona con mayor distancia mínima

        # Reacción según la zona más libre
        if best_zone == "LEFT":
            self._msg.linear.x = 0.0
            self._msg.linear.y = self._sideSpeed
            self._msg.angular.z = 0.2

        elif best_zone == "RIGHT":
            self._msg.linear.x = 0.0
            self._msg.linear.y = -self._sideSpeed
            self._msg.angular.z = -0.2

        elif best_zone in ["FRONT_LEFT", "BACK_LEFT"]:
            self._msg.linear.x = 0.0
            self._msg.linear.y = self._sideSpeed
            self._msg.angular.z = 0.2

        elif best_zone in ["FRONT_RIGHT", "BACK_RIGHT"]:
            self._msg.linear.x = 0.0
            self._msg.linear.y = -self._sideSpeed
            self._msg.angular.z = -0.2

        elif best_zone == "BACK":
            self._msg.linear.x = -self._forwardSpeed
            self._msg.linear.y = 0.0
            self._msg.angular.z = 0.2

    def stop(self):
        self._shutting_down = True
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.angular.z = 0.0
        self._cmdVel.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    robot = RobotSelfControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()


if __name__ == '__main__':
    main()