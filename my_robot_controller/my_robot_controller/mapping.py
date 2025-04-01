#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define scanning angle range
        a = 5
        safe_distance = 0.5

        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 10.0

        # Main zones
        front = np.min(np.concatenate((ranges[:a], ranges[-a:])))
        left = np.min(ranges[max(90-a, 0):min(90+a+1, len(ranges))])
        right = np.min(ranges[max(270-a, 0):min(270+a+1, len(ranges))])

        # Additional angles to avoid corners and non-parallel walls
        front_left = np.min(ranges[max(45-a, 0):min(45+a+1, len(ranges))])
        front_right = np.min(ranges[max(315-a, 0):min(315+a+1, len(ranges))])

        # *** Upgraded mapping logic ***
        if front < safe_distance:  # Obstacle in front of the robot
            if front_left < front_right:
                cmd.angular.z = -0.6  # Turn right
            else:
                cmd.angular.z = 0.6  # Turn left
            cmd.linear.x = 0.05  # Slow moving
        
        elif front_left < safe_distance and front_right < safe_distance:  
            # Inner corner → Sharp turn
            cmd.angular.z = 1.0  # Sharp left turn
            cmd.linear.x = 0.05  

        elif left < safe_distance:  # Wall on the left → Avoid
            cmd.angular.z = -0.2  # Smooth right turn
            cmd.linear.x = 0.15  

        elif right < safe_distance:  # Wall on the right → Avoid
            cmd.angular.z = 0.2  # Smooth left turn
            cmd.linear.x = 0.15  

        else:
            cmd.angular.z = 0.0  # Straight
            cmd.linear.x = 0.3  # Move forward

        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
