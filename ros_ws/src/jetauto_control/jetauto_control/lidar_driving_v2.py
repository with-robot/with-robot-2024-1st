#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

import random
import numpy as np
import cv2
from cv_bridge import CvBridge


class JetautoLidarDriving(Node):
    def __init__(self):
        super().__init__("jetauto_lidar_driving")

        # lidar msg handle nanoseconds
        self.nanoseconds = 0

        # lidar subscriber
        self.sub_lidar = self.create_subscription(LaserScan,
                                                  "jetauto/scan",
                                                  self.lidar_callback,
                                                  10)
        self.laser_scan = None

        # car control publisher
        self.pub_jetauto_car = self.create_publisher(Twist,
                                                     "jetauto_car/cmd_vel",
                                                     10)
        self.jetauto_car = Twist()

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)
        self.moving_table = {0:[0.5, -1.0], 1: [0.5, -1.0],2:[0.5, -1.0], 3:[0.5, -1.0],
                             4:[0.6, -1.0], 5:[0.6, -1.0], 6:[0.6, -1.0],7:[0.6, -1.0], 
                             8:[0.5, 0.0],
                             9:[0.6, 0.5], 10:[0.6, 1.0],11:[0.6, 1.0], 12:[0.6, 1.0], 
                             13:[0.5, 1.0],14:[0.5, 1.0], 15:[0.5, 1.0],16:[0.5, 1.0]}

    def lidar_callback(self, msg):
        self.laser_scan = msg

    def get_distances(self, scan):
        length = len(scan.ranges)
        center = length // 2
        win = length // 16  # Window size for left and right sectors

        distances = [0.0] * 17
        for i in range(8):
            distances[i] = min(scan.ranges[win * i:win * (i + 1)])
        distances[8] = scan.ranges[center]
        for i in range(9, 17):
            distances[i] = min(scan.ranges[win * i:win * (i + 1)])
        return distances

    def publish_callback(self):
        x, z = -0.1, 0.1
    
        # lidar 정보를 이용해서 x, z 결정
        if self.laser_scan:
            time = Time.from_msg(self.laser_scan.header.stamp)
            if time.nanoseconds > self.nanoseconds:
                self.nanoseconds = time.nanoseconds  # 중복 처리 방지

                # 우측 15도 4개, 중앙, 좌측 15도 4개
                distances = self.get_distances(self.laser_scan)

                d_max = max(distances)
                d_min = min(distances)
                self.get_logger().info(f'\nDistance info: {distances}')


                idx = distances.index(d_max)
                x, z = self.moving_table[idx]
                x *= 2.0
                z *= 2.0

                if d_max > 2.0:
                    if idx == 8:
                        # if abs(distances[7] - distances[9]) >= 1:
                        z += 1.0 if distances[7] < distances[9] else -1.0

                elif d_min < 0.7:
                    x = random.uniform(-6, -2)
                    
                else:
                    x += random.uniform(-4, -1)
                    z += random.uniform(-3, 3)

                if sum(distances[7:10])/3.0 < 1.0:
                    x += random.uniform(-2, 2)
                    z += random.uniform(-2, 2) 
                    self.get_logger().info(f'\nFRONT DRIVING ERROR, MODIFYING PATH')
                if sum(distances[4:7])/3.0 < 0.5 or sum(distances[:4])/4.0 < 0.5:
                    x += random.uniform(2, 5)
                    z += 5.0                      
                    self.get_logger().info(f'\nRIGHT DRIVING ERROR, MODIFYING PATH')
                if sum(distances[10:13])/3.0 < 0.5 or sum(distances[13:])/4.0 < 0.5:
                    x += random.uniform(2, 5)
                    z += -5.0
                    self.get_logger().info(f'\nLEFT DRIVING ERROR, MODIFYING PATH')    
        # publish
        driver.jetauto_car.linear.x = x
        driver.jetauto_car.angular.z = z
        self.pub_jetauto_car.publish(self.jetauto_car)


def init_ros(args=None):
    global driver
    rclpy.init(args=args) 

    driver = JetautoLidarDriving()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    init_ros(args)