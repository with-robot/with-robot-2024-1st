#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import flask


app = flask.Flask(__name__)
driver = None


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

    def lidar_callback(self, msg):
        self.laser_scan = msg
    
    def publish_callback(self):
        x, z = 0.0, 0.0
        if self.laser_scan:
            time = Time.from_msg(self.laser_scan.header.stamp)
            if time.nanoseconds > self.nanoseconds:
                self.nanoseconds = time.nanoseconds  # 중복 처리 방지

                win = 8 # 전,좌,우 센서 정보 크기
                length = len(self.laser_scan.ranges)
                center = length // 2
                distances = []
                for i in range(length // 8):
                    if i * win < center and (i+1) * win > center:
                        shift1, shift2 = 0, 1
                    elif (i+1) * win < center:
                        shift1, shift2 = 0, 0
                    else:
                        shift1, shift2 = 1, 1
                    distances.append(min(self.laser_scan.ranges[win*i+shift1:win*(i+1)+shift2]))

                self.get_logger().info(f'{distances}')
                if min(distances[6:]) > 0.5:
                    x, z = 0.1, 1.0
                elif min(distances[:4]) > 0.5:
                    x, z = 0.1, -1.0
                elif min(distances[4:6]) > 0.5:
                    x, z = 0.25, 0.0
                else:
                    x, z = -0.25, -0.1

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
