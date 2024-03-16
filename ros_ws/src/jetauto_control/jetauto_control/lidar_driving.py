#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

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

        # # depth subscriber
        # self.sub_lidar = self.create_subscription(Image,
        #                                           "jetauto/detp_camera/img_raw",
        #                                           self.depth_callback,
        #                                           10)
        # self.bridge = CvBridge()

        # car control publisher
        self.pub_jetauto_car = self.create_publisher(Twist,
                                                     "jetauto_car/cmd_vel",
                                                     10)
        self.jetauto_car = Twist()

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)
    
    def lidar_callback(self, msg):
        self.laser_scan = msg
        # self.get_logger().info(f'{msg}')
    
    # def depth_callback(self, msg):
    #     img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    #     cv2.imwrite("depth_camera.png", img)
        
    def publish_callback(self):
        x, z = -0.1, 0.1
    
        # lidar 정보를 이용해서 x, z 결정
        if self.laser_scan:
            time = Time.from_msg(self.laser_scan.header.stamp)
            if time.nanoseconds > self.nanoseconds:
                self.nanoseconds = time.nanoseconds  # 중복 처리 방지

                # 우측 15도 4개, 중앙, 좌측 15도 4개
                distances = [0.0] * 9

                length = len(self.laser_scan.ranges)
                center = length // 2
                win = length // 8 # 좌, 우 윈도우 크기

                for i in range(4):  # 우측 4개
                    distances[i] = min(self.laser_scan.ranges[win*i:win*(i+1)])
                distances[4] = self.laser_scan.ranges[center]
                for i in range(5, 9):  # 좌측 4개
                    distances[i] = min(self.laser_scan.ranges[win*i:win*(i+1)])

                # self.get_logger().info(f'{distances}')
                d_right = min(distances[0:3])
                d_center = min(distances[3:6])
                d_left = min(distances[6:9])

                if d_right > 0.5 and d_right > d_left:
                    x, z = 0.1, -1.0
                elif d_left > 0.5 and d_right < d_left:
                    x, z = 0.1, 1.0
                elif d_center > 0.5:
                    x, z = 0.25, 0.0
                else:
                    x, z = -0.25, 0.0

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
