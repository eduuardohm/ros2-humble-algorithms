import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class PotentialFieldNavigator(Node):

    def __init__(self, start, goal):
        super().__init__('potential_field_navigator')

        self.pose = None
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.ranges = []
        self.latest_scan_msg = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def scan_callback(self, msg):
        self.ranges = msg.ranges
        self.latest_scan_msg = msg

    def get_robot_position(self):
        if self.pose is None:
            return None
        pos = self.pose.position
        return np.array([pos.x, pos.y])

    def attraction_force(self, robot_pos):
        Ka = 1.0
        direction = self.goal - robot_pos
        return Ka * direction

    def repulsion_force(self, robot_pos):
        if self.latest_scan_msg is None:
            return np.zeros(2)

        ranges = self.ranges
        msg = self.latest_scan_msg
        Kr = 0.5
        influence_distance = 1.0
        total_force = np.zeros(2)

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < influence_distance:
                angle = msg.angle_min + i * msg.angle_increment
                obs_vector = np.array([math.cos(angle), math.sin(angle)])
                repulsion = Kr * (1.0 / r - 1.0 / influence_distance) * (1.0 / (r ** 2)) * obs_vector
                total_force -= repulsion

        return total_force

    def control_loop(self):
        if self.pose is None or self.ranges == []:
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return

        F_attr = self.attraction_force(robot_pos)
        F_rep = self.repulsion_force(robot_pos)
        F_total = F_attr + F_rep

        vel = Twist()
        magnitude = np.linalg.norm(F_total)
        if magnitude > 0:
            direction_angle = math.atan2(F_total[1], F_total[0])
        else:
            direction_angle = 0.0

        orientation = self.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)
        angle_diff = self.normalize_angle(direction_angle - yaw)

        vel.angular.z = 1.0 * angle_diff
        vel.linear.x = 0.2 * magnitude if abs(angle_diff) < math.pi / 4 else 0.0

        self.cmd_vel_pub.publish(vel)

    def quaternion_to_yaw(self, q):
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

    start = (-6.5, -2.0)
    goal = (6.0, -4.5)

    node = PotentialFieldNavigator(start, goal)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
