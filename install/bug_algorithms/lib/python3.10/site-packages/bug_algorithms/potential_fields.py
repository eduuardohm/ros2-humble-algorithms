import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

# Função para converter Quaternion para Euler (usaremos apenas o Yaw)
def euler_from_quaternion(quaternion):
    """
    Converte um quaternion (geometry_msgs.msg.Quaternion) para ângulos de Euler (roll, pitch, yaw).
    Retorna o yaw.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class PotentialFieldsNode(Node):

    def __init__(self, goal):
        super().__init__('potential_fields_node')

        # --- Parâmetros do Algoritmo ---
        self.K_ATT = 0.5          # Ganho da força atrativa
        self.K_REP = 1.0          # Ganho da força repulsiva
        self.OBSTACLE_RADIUS = 1.0 # Raio de influência do obstáculo (rho_0)
        self.GOAL_TOLERANCE = 0.2  # Distância para considerar que o alvo foi alcançado

        # --- Parâmetros do Robô ---
        self.MAX_LINEAR_VEL = 0.22
        self.MAX_ANGULAR_VEL = 1.0

        # --- Variáveis de Estado ---
        self.pose = None
        self.yaw = None
        self.goal = Point()
        self.goal.x = goal[0]
        self.goal.y = goal[1]
        self.latest_scan_msg = None

        # --- Publishers e Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Timer Principal ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Nó de Campos Potenciais iniciado. Indo para o alvo: ({self.goal.x}, {self.goal.y})")

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose
        self.yaw = euler_from_quaternion(self.pose.orientation)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan_msg = msg

    def control_loop(self):
        if self.pose is None or self.latest_scan_msg is None:
            self.get_logger().warn("Aguardando dados da odometria e do scan...")
            return

        # 1. Verificar se o alvo foi alcançado
        dist_to_goal = math.sqrt((self.goal.x - self.pose.position.x)**2 + (self.goal.y - self.pose.position.y)**2)
        if dist_to_goal < self.GOAL_TOLERANCE:
            self.get_logger().info("🌟 Alvo alcançado! 🌟")
            twist = Twist() # Parar o robô
            self.cmd_vel_pub.publish(twist)
            # Opcional: Desligar o nó ou definir um novo alvo
            # rclpy.shutdown()
            return

        # 2. Calcular a Força Atrativa (F_att)
        # Vetor apontando para o alvo
        att_vec_x = self.goal.x - self.pose.position.x
        att_vec_y = self.goal.y - self.pose.position.y
        # Força proporcional à distância (escalada por K_ATT)
        f_att_x = self.K_ATT * att_vec_x
        f_att_y = self.K_ATT * att_vec_y

        # 3. Calcular a Força Repulsiva Total (F_rep)
        f_rep_x = 0.0
        f_rep_y = 0.0
        
        scan_msg = self.latest_scan_msg
        for i, distance in enumerate(scan_msg.ranges):
            # Ignorar leituras inválidas ou muito distantes
            if math.isinf(distance) or math.isnan(distance) or distance > self.OBSTACLE_RADIUS:
                continue

            # Calcular a magnitude da força repulsiva para este ponto
            # A força aumenta exponencialmente à medida que a distância diminui
            magnitude_rep = self.K_REP * ((1.0 / distance) - (1.0 / self.OBSTACLE_RADIUS)) / (distance**2)

            # Calcular a direção do obstáculo no frame do mundo
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            obstacle_angle_world = self.yaw + angle

            # A força repulsiva aponta do obstáculo para o robô (direção oposta)
            f_rep_x -= magnitude_rep * math.cos(obstacle_angle_world)
            f_rep_y -= magnitude_rep * math.sin(obstacle_angle_world)

        # 4. Calcular a Força Resultante (F_total)
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        # 5. Converter a Força Resultante em Comandos de Velocidade
        twist = Twist()

        # O ângulo desejado é o ângulo do vetor da força total
        desired_angle = math.atan2(f_total_y, f_total_x)
        
        # O erro angular é a diferença entre o ângulo desejado e a orientação atual
        angle_error = desired_angle - self.yaw
        # Normalizar o erro para o intervalo [-pi, pi]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        # A velocidade angular é proporcional ao erro de ângulo
        twist.angular.z = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, angle_error * 1.5)) # Ganho P = 1.5

        # Reduz a velocidade linear ao fazer curvas acentuadas para maior estabilidade
        linear_vel_reduction = 1.0 - 0.8 * (abs(angle_error) / math.pi)
        twist.linear.x = self.MAX_LINEAR_VEL * linear_vel_reduction

        # Publicar o comando de velocidade
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Defina a posição do seu alvo aqui
    goal_position = (6.0, -4.5)

    node = PotentialFieldsNode(goal_position)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()