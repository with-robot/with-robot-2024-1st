#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation as R
import numpy as np
import flask


app = flask.Flask(__name__)
driver = None


class JetautoPickAndPlace(Node):
    def __init__(self):
        super().__init__("jetauto_pick_and_place")

        # tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publisher
        self.pub_jetauto_car = self.create_publisher(Twist, "jetauto_car/cmd_vel", 10)
        self.pub_jetauto_arm = self.create_publisher(
            JointState, "jetauto_arm/cmd_vel", 10
        )
        # car message
        self.jetauto_car = Twist()
        # arm message
        self.jetauto_arm = JointState()
        self.jetauto_arm.header.frame_id = "joint_states"
        self.jetauto_arm.name = ["joint1", "joint2", "joint3", "joint4", "r_joint"]
        self.jetauto_arm.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        # target theta
        self.jetauto_arm_target = [-np.pi / 2, -np.pi / 6, -np.pi / 6, -np.pi / 6, -np.pi / 6]

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)

    def publish_callback(self):
        curr_time = self.get_clock().now()
        self.jetauto_arm.header.stamp = curr_time.to_msg()

        # transform
        try:
            link1 = self.tf_buffer.lookup_transform(
                            'base_link',
                            'link1',
                            rclpy.time.Time())
            self.jetauto_arm.velocity[0] = self.get_theta_vel(self.jetauto_arm_target[0], link1, 2)

            link2 = self.tf_buffer.lookup_transform(
                            'link1',
                            'link2',
                            rclpy.time.Time())
            self.jetauto_arm.velocity[1] = self.get_theta_vel(self.jetauto_arm_target[1], link2, 0)

            link3 = self.tf_buffer.lookup_transform(
                            'link2',
                            'link3',
                            rclpy.time.Time())
            self.jetauto_arm.velocity[2] = self.get_theta_vel(self.jetauto_arm_target[2], link3, 0)

            link4 = self.tf_buffer.lookup_transform(
                            'link3',
                            'link4',
                            rclpy.time.Time())
            self.jetauto_arm.velocity[3] = self.get_theta_vel(self.jetauto_arm_target[3], link4, 0)

            r_link = self.tf_buffer.lookup_transform(
                            'gripper_servo_link',
                            'r_link',
                            rclpy.time.Time())
            self.jetauto_arm.velocity[4] = self.get_theta_vel(self.jetauto_arm_target[4], r_link, 0)
        except Exception as e:
            self.get_logger().info(f'{e}')
            return

        # publish
        self.pub_jetauto_car.publish(self.jetauto_car)
        self.pub_jetauto_arm.publish(self.jetauto_arm)
    
    def get_theta_vel(self, target, link, index):
        r = R.from_quat([link.transform.rotation.x,
                          link.transform.rotation.y,
                          link.transform.rotation.z,
                          link.transform.rotation.w])
        theta = r.as_euler("xyz")[index]
        # self.get_logger().info(f'{target} : {theta}')
        if abs(target - theta) < 0.05:
            return 0
        return 0.1 if target < theta else -0.1


@app.route("/jetauto_car", methods=["PUT"])
def set_jetauto_car():
    data = flask.request.get_json()
    driver.jetauto_car.linear.x = data["linear"]
    driver.jetauto_car.angular.z = data["angular"]
    return flask.jsonify({})


@app.route("/jetauto_arm", methods=["PUT"])
def set_jetauto_arm():
    data = flask.request.get_json()
    driver.get_logger().info(f'{data}')
    driver.jetauto_arm_target[0] = data["joint1"]
    driver.jetauto_arm_target[1] = data["joint2"]
    driver.jetauto_arm_target[2] = data["joint3"]
    driver.jetauto_arm_target[3] = data["joint4"]
    driver.jetauto_arm_target[4] = data["r_joint"]
    return flask.jsonify({})


def init_ros(args=None):
    global driver
    rclpy.init(args=args)

    driver = JetautoPickAndPlace()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    threading.Thread(target=lambda: init_ros(args)).start()
    app.run(host="0.0.0.0", port=5000)
