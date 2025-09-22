import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from tf_transformations import euler_from_quaternion


class Bug_Tangente(Node):
    def __init__(self):
        super().__init__('bug_tangente')

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Estado do robô
        self.state = "GOAL_SEEK"  # GOAL_SEEK ou WALL_FOLLOW
        self.pose = None
        self.ranges = []
        self.start = (1.540100, -7.914520)     # Configurar com o mesmo start do launch
        self.goal = (0, 8.7)  # Alvo (x, y) no mapa
        self.latest_scan_msg = None

        # Variáveis do bug tangente
        self.closest_point_to_goal = None
        self.min_dist_to_goal_while_following = float('inf')

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

    def distance_to_goal(self):
        if self.pose is None or self.goal is None:
            return float('inf')
        dx = self.goal[0] - self.pose.position.x
        dy = self.goal[1] - self.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    # === CALLBACKS ===
    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = msg.pose.pose
        self.yaw = yaw

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

        if dist_to_goal < 0.4:
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

            discs = self.detect_discontinuities()
            candidates = [(self.goal[0], self.goal[1])] + discs

            best_point = min(candidates, key=lambda n:
                math.dist((self.pose.position.x, self.pose.position.y), n) +
                math.dist(n, self.goal))

            if best_point == (self.goal[0], self.goal[1]) and min_front >= 0.7:
                # Caminho livre até o goal
                angle_to_goal = math.atan2(dy, dx)
                yaw = self.get_yaw()
                angle_error = self.normalize_angle(angle_to_goal - yaw)
                
                twist.linear.x = 0.3
                twist.angular.z = 0.5 * angle_error
            else:
                # Subgoal é um Oi -> troca para WALL_FOLLOW
                self.state = "WALL_FOLLOW"
                self.subgoal = best_point
                self.d_followed = self.distance_to_goal()
                self.d_reach = float('inf')
                self.get_logger().info(f"Switching to WALL_FOLLOW, subgoal={best_point}")

        elif self.state == "WALL_FOLLOW":

            current_dist_to_goal = self.distance_to_goal()
            
            IDEAL_DISTANCE = 0.5 
            SAFETY_DISTANCE = 0.8

            # Ganho proporcional para o controle de velocidade angular
            KP = 0.5 

            self.d_followed = min(self.d_followed, self.distance_to_goal())

            discs = self.detect_discontinuities()
            if discs:
                self.d_reach = min(math.dist(self.goal, d) for d in discs)

            if self.d_reach + 0.2 < self.d_followed:
                self.state = "GOAL_SEEK"
                self.get_logger().info("Critério Tangent Bug satisfeito, voltando para GOAL_SEEK.")
                return

            # Verifica se há obstáculo à frente
            if min_front < SAFETY_DISTANCE:
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # Gira para esquerda
                self.get_logger().info("Obstáculo à frente, girando para evitar.")
            
            #Se não houver obstáculo à frente, ajusta a distância da parede
            elif min_right < 1.0:
                # Calcule o erro: distância ideal - distância atual
                distance_error = IDEAL_DISTANCE - min_right
                
                # Use o erro para ajustar a velocidade angular de forma proporcional
                twist.linear.x = 0.3 # Velocidade de avanço
                twist.angular.z = KP * distance_error # O sinal negativo garante que o robô gire na direção correta
                self.get_logger().info("Ajustando distância da parede. Erro: {}".format(distance_error))

            elif min_right > 1.0 and min_front > SAFETY_DISTANCE:
                # A parede à direita "desapareceu"
                # Reduza a velocidade linear e aumente a velocidade angular para virar
                twist.linear.x = 0.0
                twist.angular.z = -0.3 
                self.get_logger().info("Curva! Perdemos a parede, virando para a direita.")

            else:
                # Não há parede detectada à direita. Gire para procurá-la.
                twist.linear.x = 0.0
                twist.angular.z = -0.3 
                self.get_logger().info("Procurando parede à direita.")

        self.cmd_vel_pub.publish(twist)

    def is_path_to_goal_clear(self):
        if self.pose is None or self.yaw is None:
            return False

        dx = self.goal[0] - self.pose.position.x
        dy = self.goal[1] - self.pose.position.y
        angle_to_goal = math.atan2(dy, dx)
        
        # Converte o ângulo para o sistema de coordenadas do robô
        relative_angle = self.normalize_angle(angle_to_goal - self.yaw)
        
        # Verifica um setor de +/- 10 graus em direção ao objetivo
        sector_start_deg = math.degrees(relative_angle) - 10
        sector_end_deg = math.degrees(relative_angle) + 10
        
        sector_ranges = self.get_sector_ranges(sector_start_deg, sector_end_deg)
        
        if not sector_ranges:
            return True # Não há leituras, o caminho está livre
        
        min_dist_in_sector = min(sector_ranges)
        
        # O caminho está livre se o obstáculo mais próximo no setor
        # do objetivo for mais distante que a distância atual até o objetivo.
        return min_dist_in_sector > self.distance_to_goal()

    def detect_discontinuities(self, threshold=0.5):
        """Detecta pontos de descontinuidade no LaserScan."""
        if not self.latest_scan_msg:
            return []

        discontinuities = []
        ranges = np.array(self.ranges)
        angle_min = self.latest_scan_msg.angle_min
        angle_increment = self.latest_scan_msg.angle_increment

        for i in range(1, len(ranges)):
            if (math.isinf(ranges[i-1]) or math.isnan(ranges[i-1]) or
                math.isinf(ranges[i]) or math.isnan(ranges[i])):
                continue
            if abs(ranges[i] - ranges[i-1]) > threshold:
                angle = angle_min + i * angle_increment
                x = self.pose.position.x + ranges[i] * math.cos(angle + self.yaw)
                y = self.pose.position.y + ranges[i] * math.sin(angle + self.yaw)
                discontinuities.append((x, y))

        return discontinuities

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
    node = Bug_Tangente()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()