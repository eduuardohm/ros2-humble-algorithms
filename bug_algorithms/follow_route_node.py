import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pandas as pd
import math
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

class FollowCSVNode(Node):
    def __init__(self, start):
        super().__init__('follow_route_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.x, self.y = start
        self.yaw = 0.0

        # csv_path = "/root/ros2_ws/src/bug_algorithms/bug_algorithms/rota_rrt.csv"

        csv_path = "/root/ros2_ws/src/bug_algorithms/bug_algorithms/rota_dstar.csv"

        self.route = pd.read_csv(
            csv_path,
            header=None,
            names=['x', 'y']
        )

        self.index = 0
        self.goal_tolerance = 0.5  # metros

        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0

        self.get_logger().info(f"Rota carregada: {len(self.route)} pontos.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def timer_callback(self):
        if self.index >= len(self.route):
            self.stop()
            self.get_logger().info("Rota concluída.")
            return

        goal_x = self.route.iloc[self.index]['x']
        goal_y = self.route.iloc[self.index]['y']

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            self.index += 1
            self.get_logger().info(f"Indo para o ponto {self.index}")
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw)

        twist = Twist()

        K_linear = 0.5
        K_angular = 1.0

        if abs(angle_diff) > 0.1:
            twist.angular.z = max(-self.max_angular_speed,
                                  min(self.max_angular_speed, K_angular * angle_diff))
            twist.linear.x = 0.0
        else:
            twist.linear.x = min(self.max_linear_speed, K_linear * distance)
            twist.angular.z = max(-self.max_angular_speed,
                                  min(self.max_angular_speed, K_angular * angle_diff))

        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        # Mantém o ângulo entre -pi e pi
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    spawn = (3.0, -3.0)
    node = FollowCSVNode(spawn)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()