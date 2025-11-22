import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random


class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Paràmetres Configurables
        self.declare_parameter('distance_limit', 0.3)      # Distància mínima davant d'un obstacle per reaccionar
        self.declare_parameter('speed_factor', 1.0)        # Factor multiplicatiu de velocitat
        self.declare_parameter('forward_speed', 0.2)       # Velocitat cap endavant
        self.declare_parameter('lateral_speed', 0.2)       # Velocitat lateral (esquerra/dreta)
        self.declare_parameter('rotation_speed', 0.3)      # Velocitat de rotació
        self.declare_parameter('time_to_stop', 5.0)        # Temps total d’execució abans de parar

        # Assignem els valors dels paràmetres a variables internes
        self._distanceLimit = self.get_parameter('distance_limit').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._lateralSpeed = self.get_parameter('lateral_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value
        
        # Missatge de velocitat que s’enviarà al robot
        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.linear.y = 0.0
        self._msg.angular.z = 0.0

        # Publicador de velocitat
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer que crida timer_callback cada 0.05s
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Subscripció al topic del LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10  # Default QoS depth
        )
        
        # Estat de la màquina de control
        self.state = "NORMAL"         # Estats possibles: NORMAL, AVOID_BACKWARD, AVOID_LATERAL
        self.state_start = 0.0        # Temps d’inici de cada estat
        self.backward_duration = 0.5  # Temps que retrocedeix quan detecta obstacle
        self.lateral_duration = 0.7   # Temps que es mou lateralment

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._last_info_time = self.start_time
        self._last_speed_time = self.start_time



    # Funció que s’executa periòdicament (cada 0.05s)
    def timer_callback(self):
        if self._shutting_down:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time = now_sec - self.start_time

        # Publica el missatge de velocitat al robot
        self._cmdVel.publish(self._msg)

        # Mostrar informació per consola un cop per segon
        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(
                f"Vx: {self._msg.linear.x:.2f} m/s, Vy: {self._msg.linear.y:.2f} m/s, "
                f"W: {self._msg.angular.z:.2f} rad/s | State: {self.state} | Time: {elapsed_time:.1f}s"
            )
            self._last_speed_time = now_sec

        # Atura el robot si s’ha arribat al temps límit
        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info("Robot stopped")
            rclpy.try_shutdown()


    # Funció que processa les dades del LIDAR
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        # Convertim angles de radians a graus
        angle_min_deg = scan.angle_min * 180.0 / math.pi
        angle_increment_deg = scan.angle_increment * 180.0 / math.pi

        # Filtrar lectures vàlides dins del rang [-150°, 150°]
        valid_ranges = []
        for i, distance in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_increment_deg
            if angle_deg > 180.0:
                angle_deg -= 360.0
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            if -150 < angle_deg < 150:
                valid_ranges.append((distance, angle_deg))

        if not valid_ranges:
            return

        # Dividim les distàncies segons zona
        front_dists = [d for d,a in valid_ranges if -45 <= a <= 45]
        left_dists  = [d for d,a in valid_ranges if 45 < a <= 110]
        right_dists = [d for d,a in valid_ranges if -110 <= a < -45]
        back_dists  = [d for d,a in valid_ranges if a > 110 or a < -110]

        # Distància representativa mínima per cada zona
        d_front = min(front_dists) if front_dists else 10.0
        d_left  = min(left_dists)  if left_dists  else 10.0
        d_right = min(right_dists) if right_dists else 10.0
        d_back  = min(back_dists)  if back_dists  else 10.0

        # Mostrar informació del LIDAR un cop per segon
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_info_time >= 1:
            self.get_logger().info(
                f"[SCAN] FRONT:{d_front:.2f} | LEFT:{d_left:.2f} | RIGHT:{d_right:.2f} | BACK:{d_back:.2f}"
            )
            self._last_info_time = now

        # Lògica per evitar obstacles amb moviment holonòmic
        threshold = 0.1  # Diferència mínima per decidir direcció
        min_distance_for_rotation = 0.5  # Si totes les distàncies són petites, gira

        if self.state == "NORMAL":
            # Moure cap a la zona amb més espai
            if min(d_front, d_left, d_right, d_back) < min_distance_for_rotation:
                # Si està molt tancat → gir aleatori
                self._msg.linear.x = 0.0
                self._msg.linear.y = 0.0
                self._msg.angular.z = random.choice([-1, 1]) * self._rotationSpeed
            else:
                # Moviment lateral
                if abs(d_left - d_right) < threshold:
                    self._msg.linear.y = self._lateralSpeed * random.choice([-1,1])
                else:
                    self._msg.linear.y = self._lateralSpeed if d_left > d_right else -self._lateralSpeed
                # Moviment endavant/endreta
                if abs(d_front - d_back) < threshold:
                    self._msg.linear.x = self._forwardSpeed * random.choice([-1,1])
                else:
                    self._msg.linear.x = self._forwardSpeed if d_front > d_back else -self._forwardSpeed

                self._msg.linear.x *= self._speedFactor
                self._msg.linear.y *= self._speedFactor
                self._msg.angular.z = 0.0

                # Si obstacle molt proper al davant → retrocedir
                if d_front < self._distanceLimit:
                    self.state = "AVOID_BACKWARD"
                    self.state_start = now
                    self.get_logger().info("STATE → AVOID_BACKWARD")

        elif self.state == "AVOID_BACKWARD":
            # Retrocedim
            self._msg.linear.x = -self._forwardSpeed * self._speedFactor
            self._msg.linear.y = 0.0
            self._msg.angular.z = 0.0

            # Passar a moviment lateral després de backward_duration
            if now - self.state_start >= self.backward_duration:
                self.state = "AVOID_LATERAL"
                self.state_start = now
                self.get_logger().info("STATE → AVOID_LATERAL")

        elif self.state == "AVOID_LATERAL":
            # Moviment lateral per esquivar obstacle
            self._msg.linear.x = 0.0
            self._msg.angular.z = 0.0

            if d_left > d_right:
                self._msg.linear.y = +self._lateralSpeed
            else:
                self._msg.linear.y = -self._lateralSpeed

            # Tornar a estat NORMAL després de lateral_duration
            if now - self.state_start >= self.lateral_duration:
                self.state = "NORMAL"
                self.get_logger().info("STATE → NORMAL")


    # Funció per aturar el robot completament
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