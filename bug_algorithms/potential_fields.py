import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv

class PotentialFieldNavigator(Node):

    def __init__(self, csv_path='rota_rrt.csv', goal_tolerance=0.25):
        super().__init__('rrt_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.follow_path)
        self.pose = None
        self.goal_tolerance = goal_tolerance

        # Carrega caminho do CSV
        self.path = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                self.path.append([float(row[0]), float(row[1])])
        self.next_wp = 0

        print(f"ROTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA: {csv_path}")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        print(self.pose)

    def follow_path(self):
        if self.pose is None or self.next_wp >= len(self.path):
            return

        x = self.pose.position.x
        y = self.pose.position.y

        print(f'Posição: ({x}, {y})')

        # Próximo waypoint
        wx, wy = self.path[self.next_wp]

        # Distância ao waypoint
        dx = wx - x
        dy = wy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            self.next_wp += 1
            if self.next_wp >= len(self.path):
                self.cmd_pub.publish(Twist())  # stop
            return

        # Ângulo até o waypoint
        yaw = self.get_yaw(self.pose.orientation)
        angle_to_wp = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_wp - yaw)

        # Proporcional simples para seguir rota
        twist = Twist()
        twist.linear.x = 0.2 if abs(angle_diff) < 0.5 else 0.0
        twist.angular.z = 1.0 * angle_diff
        self.cmd_pub.publish(twist)

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    # start = (-6.5, -2.0)
    # goal = (6.0, -4.5)
    node = PotentialFieldNavigator(start, goal)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
