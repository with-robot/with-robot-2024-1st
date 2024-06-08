#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np


driver = None


class JetautoMapping(Node):
    def __init__(self):
        super().__init__("jetauto_mapping")

        # lidar msg handle nanoseconds
        self.lidar_nanoseconds = 0
        self.odometry_nanoseconds = 0
        #self.seconds = 0

        # lidar subscriber
        self.sub_lidar = self.create_subscription(
            LaserScan,
            "jetauto/scan",
            self.lidar_callback,
            10,
        )
        self.laser_scan = None

        # odometry subscriber
        self.sub_odom = self.create_subscription(
            TwistStamped,
            "unity_tf",
            self.odom_callback,
            10,
        )
        self.odometry = None

#        # car control publisher
#        self.pub_jetauto_car = self.create_publisher(
#            Twist,
#            "jetauto_car/cmd_vel",
#            10,
#        )
#        self.jetauto_car = Twist()
#
#        # timer
#        self.timer = self.create_timer(0.1, self.publish_callback)

#        self.measurement_txt = open("./measurement.txt", "w")
#        self.pose_txt = open("./poses.txt", "w")
#
#        self.image = np.zeros([500, 500], dtype=np.uint8)

#    def __del__(self):
#        super().__del__()
#        if not self.measurement_txt.closed:
#            self.measurement_txt.close()
#        if not self.pose_txt.closed:
#            self.pose_txt.close()

    def lidar_callback(self, msg):
        self.laser_scan = msg
        #self.get_logger().info(f'{msg}')

        # Run if only laser scan from simulation is updated
        if self.laser_scan:
            time = Time.from_msg(self.laser_scan.header.stamp)
            if time.nanoseconds > self.lidar_nanoseconds:
                self.lidar_nanoseconds = time.nanoseconds

                # Write sensor measurement data for mapping
                self.measurement_txt.write(f"{time.nanoseconds} ")
                length = len(self.laser_scan.ranges)
                for i in range(length):
                    self.measurement_txt.write(f"{self.laser_scan.ranges[i]} ")
                self.measurement_txt.write(f"\n")

    def odom_callback(self, msg):
        self.odometry = msg
        #self.get_logger().info(f'{msg}')

        # Run if only odometry from simulation is updated
        if self.odometry:
            time = Time.from_msg(self.odometry.header.stamp)
            if time.nanoseconds > self.odometry_nanoseconds:
                self.odometry_nanoseconds = time.nanoseconds

                # Write odeomtry data for mapping
                self.pose_txt.write(f"{time.nanoseconds} ")
                self.pose_txt.write(f"{self.odometry.twist.linear.x} ")
                self.pose_txt.write(f"{self.odometry.twist.linear.y} ")
                self.pose_txt.write(f"{self.odometry.twist.linear.z} ")
                self.pose_txt.write(f"{self.odometry.twist.angular.x} ")
                self.pose_txt.write(f"{self.odometry.twist.angular.y} ")
                self.pose_txt.write(f"{self.odometry.twist.angular.z} ")
                self.pose_txt.write(f"\n")

#    def publish_callback(self):
#        x, z = 0.0, 0.0
#        if self.laser_scan:
#            time = Time.from_msg(self.laser_scan.header.stamp)
#            # Run if only laser scan from simulation is updated
#            if time.nanoseconds > self.nanoseconds:
#                self.nanoseconds = time.nanoseconds
#
##                ### Driving ###
##                win = 8 # # of elements in forward, left, right sensor groups
##                length = len(self.laser_scan.ranges)
##                center = length // 2
##                distances = []
##                for i in range(length // 8):
##                    if i * win < center and (i+1) * win > center:
##                        shift1, shift2 = 0, 1
##                    elif (i+1) * win < center:
##                        shift1, shift2 = 0, 0
##                    else:
##                        shift1, shift2 = 1, 1
##                    distances.append(min(self.laser_scan.ranges[win*i+shift1:win*(i+1)+shift2]))
##
##                #self.get_logger().info(f'{distances}')
##                if min(distances[6:]) > 0.5:
##                    x, z = 0.1, 1.0
##                elif min(distances[:4]) > 0.5:
##                    x, z = 0.1, -1.0
##                elif min(distances[4:6]) > 0.5:
##                    x, z = 0.25, 0.0
##                else:
##                    x, z = -0.25, -0.1
##                ### End driving ###
##
#
#                with open(self.pose_txt, "a") as fp:
#                    # TODO: measurement time and pose time can be different
#                    # But, for now, ignore it.
#                    fp.write(f"{time.nanoseconds} ")
#                    fp.write(f"{self.odometry.twist.linear.x} ")
#                    fp.write(f"{self.odometry.twist.linear.y} ")
#                    fp.write(f"{self.odometry.twist.linear.z} ")
#                    fp.write(f"{self.odometry.twist.angular.x} ")
#                    fp.write(f"{self.odometry.twist.angular.y} ")
#                    fp.write(f"{self.odometry.twist.angular.z} ")
#                    fp.write(f"\n")
#
##                seconds, _ = time.seconds_nanoseconds()
##                self.get_logger().info(f'{seconds}')
#
##            if time.seconds_nanoseconds()[0] > self.seconds:
##                self.seconds = time.seconds_nanoseconds()[0]
##                y = int(self.odometry.twist.linear.y * 100) + 100
##                x = int(self.odometry.twist.linear.x * 100) + 250
##                self.image[y, x] = 255
##                cv2.imwrite("pose_map.png", self.image)
#
##        # publish
##        driver.jetauto_car.linear.x = x
##        driver.jetauto_car.angular.z = z
##        self.pub_jetauto_car.publish(self.jetauto_car)


def init_ros(args=None):
    global driver
    rclpy.init(args=args)

    driver = JetautoMapping()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    init_ros(args)


