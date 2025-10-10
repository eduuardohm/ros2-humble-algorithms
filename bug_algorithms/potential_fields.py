#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math

class PotentialFields(Node):
    def __init__(self):
        super().__init__('potential_fields')

        # === Publishers/Subscribers ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # === Control loop ===
        self.timer = self.create_timer(0.1, self.control_loop)

        # === Variables ===
        self.map_data = None
        self.pose = None
        self.goal = np.array([3.0, 3.0])  # posição-alvo (x, y)
        self.k_att = 1.0
        self.k_rep = 100.0
        self.obs_threshold = 0.5  # distância de influência do obstáculo

        self.get_logger().info("Nodo Potential Fields iniciado.")

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        width, height = msg.info.width, msg.info.height
        self.map = np.array(msg.data).reshape(height, width)
        self.get_logger().info("Mapa recebido!")

    def odom_callback(self, msg):
        self.pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def compute_potential_field(self, position):
        # Campo atrativo
        F_att = -self.k_att * (position - self.goal)

        # Campo repulsivo
        F_rep = np.zeros(2)
        if self.map_data is not None:
            for y in range(self.map.shape[0]):
                for x in range(self.map.shape[1]):
                    if self.map[y, x] > 50:  # obstáculo
                        obs_pos = self.origin + np.array([x, y]) * self.resolution
                        diff = position - obs_pos
                        dist = np.linalg.norm(diff)
                        if dist < self.obs_threshold and dist > 1e-6:
                            F_rep += self.k_rep * ((1.0 / dist) - (1.0 / self.obs_threshold)) * (1.0 / (dist ** 3)) * diff

        return F_att + F_rep

    def control_loop(self):
        if self.pose is None or self.map_data is None:
            return

        # Calcula força total
        F = self.compute_potential_field(self.pose)

        # Define velocidades proporcionais ao campo
        vel_msg = Twist()
        vel_msg.linear.x = 0.2 * np.clip(np.linalg.norm(F), 0.0, 1.0)
        vel_msg.angular.z = 2.0 * math.atan2(F[1], F[0])

        # Publica
        self.cmd_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFields()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
