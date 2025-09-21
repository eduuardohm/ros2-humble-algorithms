import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Bug1(Node):
    def __init__(self):
        super().__init__('bug1')

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Estado do robô
        self.state = "GOAL_SEEK"  # GOAL_SEEK ou WALL_FOLLOW
        self.pose = None
        self.ranges = []
        self.goal = (0, 8.7)  # Alvo (x, y) no mapa
        self.hit_point = None
        self.latest_scan_msg = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def angle_to_index(self, angle_rad):
        if self.latest_scan_msg is None:
            return 0

        msg = self.latest_scan_msg
        # Normaliza o ângulo dentro do range [angle_min, angle_max]
        angle_rad = (angle_rad + 2 * math.pi) % (2 * math.pi)
        angle_min = (msg.angle_min + 2 * math.pi) % (2 * math.pi)

        index = int((angle_rad - angle_min) / msg.angle_increment)

        return max(0, min(index, len(msg.ranges) - 1))

    def get_sector_ranges(self, start_angle_deg, end_angle_deg):
        """Obtém leituras de um setor angular específico em graus"""
        if self.latest_scan_msg is None:
            return []
        
        start_idx = self.angle_to_index(math.radians(start_angle_deg))
        end_idx = self.angle_to_index(math.radians(end_angle_deg))
        
        if start_idx <= end_idx:
            sector = self.ranges[start_idx:end_idx+1]
        else:
            sector = self.ranges[start_idx:] + self.ranges[:end_idx+1]
        
        # Filtra valores inválidos
        return [r for r in sector if not (math.isinf(r) or math.isnan(r)) and 0.0 < r < 3.5]

    # === CALLBACKS ===
    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.ranges = msg.ranges
        self.latest_scan_msg = msg

        # Log de distâncias em direções principais
        angles_deg = {"front": 0, "left": 90, "right": -90, "back": 180}

        for label, angle_deg in angles_deg.items():
            angle_rad = math.radians(angle_deg)
            index = self.angle_to_index(angle_rad)

            # Pega valor e trata possíveis erros
            dist = msg.ranges[index]
            if math.isinf(dist) or math.isnan(dist):
                dist_str = "inf/nan"
            else:
                dist_str = f"{dist:.2f} m"

            self.get_logger().info(f"{label.capitalize()} ({angle_deg}°): index {index}, distance: {dist_str}")

    # === LÓGICA DE CONTROLE ===
    def control_loop(self):
        if self.pose is None or not self.ranges:
            return

        twist = Twist()

        # Distância ao alvo
        dx = self.goal[0] - self.pose.position.x
        dy = self.goal[1] - self.pose.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"Current State: {self.state}, Distance to Goal: {dist_to_goal:.2f} m")

        if dist_to_goal < 0.2:
            self.get_logger().info("CHEGUEI NESTA PORRA!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        # Obter setores do laser
        front_sector = self.get_sector_ranges(-30, 30)  # ±20 graus na frente
        left_sector = self.get_sector_ranges(30, 80)    # 30-80 graus à esquerda
        right_sector = self.get_sector_ranges(-80, -30) # 30-80 graus à direita

        min_front = min(front_sector) if front_sector else float('inf')
        min_left = min(left_sector) if left_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')

        if self.state == "GOAL_SEEK":
            if min_front < 1.2:  # Obstáculo muito próximo
                self.state = "WALL_FOLLOW"
                self.hit_point = (self.pose.position.x, self.pose.position.y)
                self.wall_follow_direction = "left"
                self.get_logger().info(f"Switching to WALL_FOLLOW, following {self.wall_follow_direction}")
            else:
                # Navegação normal para o goal
                angle_to_goal = math.atan2(dy, dx)
                yaw = self.get_yaw()
                angle_error = self.normalize_angle(angle_to_goal - yaw)
                
                twist.linear.x = 0.25
                twist.angular.z = 0.3 * angle_error

        elif self.state == "WALL_FOLLOW":
            
            if min_front > 1.2:
                self.state = "GOAL_SEEK"
                self.get_logger().info("Returning to GOAL_SEEK")
                return

            if min_front < 0.5:  # Obstáculo muito próximo à frente
                twist.linear.x = -0.05  # Rézinha massa
                twist.angular.z = 0.7  # Gira para a direita
                self.get_logger().info("Obstacle ahead, turning right")
            elif min_left < 0.5:  # Muito perto da parede
                twist.linear.x = 0.05
                twist.angular.z = -1.0  # Afasta da parede
                self.get_logger().info("Muito perto da parede lateral, afastando.")
            elif min_left > 0.8:  # Muito longe da parede
                twist.linear.x = 0.05
                twist.angular.z = 0.1  # Aproxima da parede
                self.get_logger().info("Muito longe da parede lateral, aproximando.")
            else:  # Distância adequada
                twist.linear.x = 0.15
                twist.angular.z = 0.0
                self.get_logger().info("Muito longe da parede lateral, aproximando.")

        self.cmd_vel_pub.publish(twist)

    def get_yaw(self):
        q = self.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normaliza ângulo para o range [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = Bug1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()