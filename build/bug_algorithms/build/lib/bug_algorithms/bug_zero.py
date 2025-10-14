import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class Bug_Zero(Node):

    def __init__(self, start, goal):
        super().__init__('bug_zero')

        self.state = "GOAL_SEEK"
        self.pose = None
        self.start = start
        self.goal = goal
        self.ranges = []
        self.latest_scan_msg = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.ranges = msg.ranges
        self.latest_scan_msg = msg

    def angle_to_index(self, angle):
        if self.latest_scan_msg is None:
            return 0
        
        msg = self.latest_scan_msg
        angle_rad = math.radians(angle)

        angle_rad = (angle_rad + 2 * math.pi) % (2 * math.pi)
        angle_min = (msg.angle_min + 2 * math.pi) % (2 * math.pi)

        index = int((angle_rad - angle_min) / msg.angle_increment)

        return max(0, min(index, len(msg.ranges) - 1))

    def get_sector_ranges(self, start_angle_deg, end_angle_deg):
        if self.latest_scan_msg is None:
            return []

        start_index = self.angle_to_index(start_angle_deg)
        end_index = self.angle_to_index(end_angle_deg)

        if start_index <= end_index:
            sector = self.ranges[start_index:end_index+1]
        else:
            sector = self.ranges[start_index:] + self.ranges[:end_index]

        return [r for r in sector if not (math.isinf(r) or math.isnan(r)) and 0.0 < r < 3.5]

    def control_loop(self):
        if self.pose is None or not self.ranges:
            return

        twist = Twist()

        SAFETY_DISTANCE = 0.5
        IDEAL_DISTANCE = 1.0
        TANGENT_DISTANCE = 1.5
        
        KP = 0.1

        front_sector = self.get_sector_ranges(-20, 20)
        right_sector = self.get_sector_ranges(-110, -70)
        fr_sector = self.get_sector_ranges(-35, -20)
        left_sector = self.get_sector_ranges(70, 110)
        fl_sector = self.get_sector_ranges(20, 35)
        
        min_front = min(front_sector) if front_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')
        min_fr = min(fr_sector) if fr_sector else float('inf')
        min_left = min(left_sector) if left_sector else float('inf')
        min_fl = min(fl_sector) if fl_sector else float('inf')

        if min_front <= TANGENT_DISTANCE or min_left <= TANGENT_DISTANCE or min_right <= TANGENT_DISTANCE:
            self.state = "TANGENT_WALL"
            self.tangent_wall(twist)
            
        # if min_front < IDEAL_DISTANCE + 0.25:
        #     self.get_logger().info("Obstaculo a frente")

        #     if min_fr < min_fl:
        #         twist.angular.z = -0.5
        #     else:
        #         twist.angular.z = 0.5
            
        #     twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

    def tangent_wall(self, twist):

        SAFETY_DISTANCE = 0.5
        IDEAL_DISTANCE = 1.0
        TANGENT_DISTANCE = 1.25
        
        KP = 0.1

        front_sector = self.get_sector_ranges(-20, 20)
        right_sector = self.get_sector_ranges(-110, -70)
        fr_sector = self.get_sector_ranges(-35, -20)
        rr_sector = self.get_sector_ranges(-160, -110)
        left_sector = self.get_sector_ranges(70, 110)
        fl_sector = self.get_sector_ranges(20, 35)
        rl_sector = self.get_sector_ranges(110, 160)
        
        min_front = min(front_sector) if front_sector else float('inf')
        min_right = min(right_sector) if right_sector else float('inf')
        min_fr = min(fr_sector) if fr_sector else float('inf')
        min_rr = min(rr_sector) if rr_sector else float('inf')
        min_left = min(left_sector) if left_sector else float('inf')
        min_fl = min(fl_sector) if fl_sector else float('inf')
        min_rl = min(rl_sector) if rl_sector else float('inf')

        if min_left < min_right:

            self.get_logger().info("Obstaculo a esquerda")
            self.get_logger().info(f"min_rl: {min_rl}")

            if min_left < IDEAL_DISTANCE + 0.25:
                self.get_logger().info("Obstaculo a esquerda")

                distance_error =  min_left - IDEAL_DISTANCE
                angular_z = KP * distance_error

                twist.angular.z = angular_z
                twist.linear.x = 0.1
            else:
                self.get_logger().info("Espaço livre a esquerda")

                distance_error =  min_rl - IDEAL_DISTANCE
                angular_z = KP * distance_error * 5

                twist.angular.z = angular_z
                twist.linear.x = 0.2

        else:

            self.get_logger().info("Obstaculo a direita")
            self.get_logger().info(f"min_rr: {min_rr}")

            if min_right < IDEAL_DISTANCE + 0.25:
                self.get_logger().info("Obstaculo a direita")

                distance_error = min_right - IDEAL_DISTANCE
                angular_z = -KP * distance_error

                twist.angular.z = angular_z
                twist.linear.x = 0.1
            else:
                self.get_logger().info("Espaço livre a direita")

                distance_error =  min_rr - IDEAL_DISTANCE
                angular_z = -KP * distance_error * 5

                twist.angular.z = angular_z
                twist.linear.x = 0.2


        self.cmd_vel_pub.publish(twist)
        return

def main(args=None):
    rclpy.init(args=args)

    start = (1.5, -7.0)
    goal = (0, 8.7)

    node = Bug_Zero(start, goal)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()