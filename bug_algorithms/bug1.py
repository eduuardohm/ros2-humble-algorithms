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
        self.start = (1.540100, -7.914520)     # Configurar com o mesmo start do launch
        self.goal = (0, 8.7)  # Alvo (x, y) no mapa
        self.hit_point = None
        self.latest_scan_msg = None
        self.has_left_hit_point = False

        # Variáveis do bug1
        self.closest_point_to_goal = None
        self.has_made_a_full_tour = False
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
            if min_front < 0.7:  # Obstáculo muito próximo
            # Caso esteja próximo de alguma parede

                self.state = "WALL_FOLLOW"
                self.hit_point = (self.pose.position.x, self.pose.position.y)
                self.min_dist_to_goal_while_following = self.distance_to_goal()
                self.closest_point_to_goal = self.hit_point
                self.has_made_a_full_tour = False
                self.has_left_hit_point = False
                self.get_logger().info("Switching to WALL_FOLLOW")
            else:
                # Navegação normal para o goal
                angle_to_goal = math.atan2(dy, dx)
                yaw = self.get_yaw()
                angle_error = self.normalize_angle(angle_to_goal - yaw)
                
                twist.linear.x = 0.3
                twist.angular.z = 0.5 * angle_error

        elif self.state == "WALL_FOLLOW":

            current_dist_to_goal = self.distance_to_goal()

            if current_dist_to_goal < self.min_dist_to_goal_while_following:
                self.min_dist_to_goal_while_following = current_dist_to_goal
                self.closest_point_to_goal = (self.pose.position.x, self.pose.position.y)
                self.get_logger().info(f"New closest point to goal: {self.closest_point_to_goal} with distance {self.min_dist_to_goal_while_following:.2f} m")

            # Lógica de retorno ao hit_point
            dist_to_hit_point = math.sqrt(
                (self.pose.position.x - self.hit_point[0])**2 +
                (self.pose.position.y - self.hit_point[1])**2
            )

            dist_to_closest_point = math.sqrt(
                (self.pose.position.x - self.closest_point_to_goal[0])**2 +
                (self.pose.position.y - self.closest_point_to_goal[1])**2
            )

            if not self.has_left_hit_point and dist_to_hit_point > 0.3: # Use uma distância pequena
                self.has_left_hit_point = True
                self.get_logger().info("Robô se afastou do ponto de impacto, pode iniciar a verificação de retorno.")

            if self.has_left_hit_point and dist_to_hit_point < 0.2 and not self.has_made_a_full_tour:
                # Robô retornou ao hit_point
                self.get_logger().info("Voltando ao ponto de impacto.")
                self.has_made_a_full_tour = True

            if self.has_made_a_full_tour and dist_to_closest_point < 0.5:
                self.state = "GOAL_SEEK"
                self.get_logger().info("Chegou ao ponto mais próximo. Retornando para GOAL_SEEK.")
                self.hit_point = None
                self.closest_point_to_goal = None
                self.has_left_hit_point = False
                self.min_dist_to_goal_while_following = float('inf')
                self.has_made_a_full_tour = False
                return
            
            IDEAL_DISTANCE = 0.5 
            SAFETY_DISTANCE = 0.8

            # Ganho proporcional para o controle de velocidade angular
            KP = 0.5 

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