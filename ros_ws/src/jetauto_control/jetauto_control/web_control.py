#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import flask


app = flask.Flask(__name__)
driver = None


class JetautoWebControl(Node):
    def __init__(self):
        super().__init__("jetauto_control_web")

        # publisher
        self.pub_jetauto_car = self.create_publisher(Twist, "jetauto_car/cmd_vel", 10)
        self.pub_jetauto_arm = self.create_publisher(
            JointState, "jetauto_arm/cmd_vel", 10
        )

        self.jetauto_car = Twist()
        self.jetauto_arm = JointState()
        self.jetauto_arm.header.frame_id = "joint_states"
        self.jetauto_arm.name = ["joint1", "joint2", "joint3", "joint4", "r_joint"]
        self.jetauto_arm.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)

    def publish_callback(self):
        curr_time = self.get_clock().now()
        self.jetauto_arm.header.stamp = curr_time.to_msg()

        # publish
        self.pub_jetauto_car.publish(self.jetauto_car)
        self.pub_jetauto_arm.publish(self.jetauto_arm)
        self.get_logger().info(f"{self.jetauto_arm}")


@app.route("/jetauto_car", methods=["PUT"])
def set_jetauto_car():
    data = flask.request.get_json()
    driver.jetauto_car.linear.x = data["linear"]
    driver.jetauto_car.angular.z = data["angular"]
    return flask.jsonify({})


@app.route("/jetauto_arm", methods=["PUT"])
def set_jetauto_arm():
    data = flask.request.get_json()
    driver.jetauto_arm.velocity[0] = data["joint1"]
    driver.jetauto_arm.velocity[1] = data["joint2"]
    driver.jetauto_arm.velocity[2] = data["joint3"]
    driver.jetauto_arm.velocity[3] = data["joint4"]
    driver.jetauto_arm.velocity[4] = data["r_joint"]
    return flask.jsonify({})


def init_ros(args=None):
    global driver
    rclpy.init(args=args)

    driver = JetautoWebControl()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


def main(args=None):
    threading.Thread(target=lambda: init_ros(args)).start()
    app.run(host="0.0.0.0", port=5000)
