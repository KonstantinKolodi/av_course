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
        a = 5  # Увеличил диапазон для более стабильных измерений
        safe_distance = 0.5  # Минимальная дистанция от стены

        # Обработка данных с LiDAR (замена inf на 10 м)
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 10.0

        # Основные зоны
        front = np.min(np.concatenate((ranges[:a], ranges[-a:])))
        left = np.min(ranges[max(90-a, 0):min(90+a+1, len(ranges))])
        right = np.min(ranges[max(270-a, 0):min(270+a+1, len(ranges))])

        # Дополнительные углы для детекции внешних и внутренних углов
        front_left = np.min(ranges[max(45-a, 0):min(45+a+1, len(ranges))])
        front_right = np.min(ranges[max(315-a, 0):min(315+a+1, len(ranges))])

        # *** Улучшенная логика движения ***
        if front < safe_distance:  # Препятствие прямо перед роботом
            if front_left < front_right:
                cmd.angular.z = -0.6  # Разворот вправо
            else:
                cmd.angular.z = 0.6  # Разворот влево
            cmd.linear.x = 0.05  # Медленное движение вперед (чтобы не остановился)
        
        elif front_left < safe_distance and front_right < safe_distance:  
            # Внутренний угол → Резко поворачиваем
            cmd.angular.z = 1.0  # Резкий левый поворот
            cmd.linear.x = 0.05  

        elif left < safe_distance:  # Стена слева → Отдаляемся
            cmd.angular.z = -0.2  # Плавный поворот вправо
            cmd.linear.x = 0.15  

        elif right < safe_distance:  # Стена справа → Отдаляемся
            cmd.angular.z = 0.2  # Плавный поворот влево
            cmd.linear.x = 0.15  

        else:
            cmd.angular.z = 0.0  # Держим ровный курс
            cmd.linear.x = 0.3  # Двигаемся вперед

        # Отправка команды
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
