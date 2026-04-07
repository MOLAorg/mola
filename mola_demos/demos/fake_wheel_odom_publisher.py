#!/usr/bin/env python3
"""
Publishes fake wheel odometry on /wheel_odom as nav_msgs/Odometry.

Simulates a robot driving in a circle at 1 m/s with 0.2 rad/s yaw rate,
with configurable Gaussian noise and systematic drift.

Usage:
  ros2 run mola_state_estimation_smoother fake_wheel_odom_publisher.py
  # or directly:
  python3 fake_wheel_odom_publisher.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class FakeWheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_wheel_odom_publisher')

        # Parameters
        self.declare_parameter('topic', '/wheel_odom')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('noise_xy', 0.02)       # m per step
        self.declare_parameter('noise_yaw', 0.005)     # rad per step
        self.declare_parameter('initial_pose_noise_xy', 1.0)    # m
        self.declare_parameter('initial_pose_noise_yaw', 1.0)   # rad
        self.declare_parameter('drift_scale_x', 1.02)  # 2% systematic drift
        self.declare_parameter('vx', 1.0)              # m/s ground truth
        self.declare_parameter('wz', 0.2)              # rad/s ground truth
        self.declare_parameter('frame_id', 'odom_wheels')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)

        topic = self.get_parameter('topic').value
        rate = self.get_parameter('rate_hz').value
        self.noise_xy = self.get_parameter('noise_xy').value
        self.noise_yaw = self.get_parameter('noise_yaw').value
        self.initial_pose_noise_xy = self.get_parameter(
            'initial_pose_noise_xy').value
        self.initial_pose_noise_yaw = self.get_parameter(
            'initial_pose_noise_yaw').value
        self.drift_scale_x = self.get_parameter('drift_scale_x').value
        self.vx = self.get_parameter('vx').value
        self.wz = self.get_parameter('wz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.do_publish_tf = self.get_parameter('publish_tf').value

        self.pub = self.create_publisher(Odometry, topic, 10)
        if self.do_publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 1.0 / rate
        self.x = np.random.normal(0, self.initial_pose_noise_xy)
        self.y = np.random.normal(0, self.initial_pose_noise_xy)
        self.yaw = np.random.normal(0, self.initial_pose_noise_yaw)
        self.step = 0

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f'Publishing fake wheel odom on {topic} at {rate} Hz '
            f'(noise_xy={self.noise_xy}, drift={self.drift_scale_x})')

    def timer_callback(self):
        if self.step > 0:
            # Ground truth delta
            dx_gt = self.vx * self.dt
            dyaw_gt = self.wz * self.dt

            # Apply systematic drift + noise
            dx = dx_gt * self.drift_scale_x + \
                np.random.normal(0, self.noise_xy)
            dy = np.random.normal(0, self.noise_xy)
            dyaw = dyaw_gt + np.random.normal(0, self.noise_yaw)

            # Integrate in local frame
            self.x += dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
            self.y += dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            self.yaw += dyaw

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        # Covariance (diagonal)
        cov = [0.0] * 36
        cov[0] = self.noise_xy ** 2   # xx
        cov[7] = self.noise_xy ** 2   # yy
        cov[14] = self.noise_xy ** 2  # zz
        cov[21] = self.noise_yaw ** 2  # rotX
        cov[28] = self.noise_yaw ** 2  # rotY
        cov[35] = self.noise_yaw ** 2  # yaw-yaw
        msg.pose.covariance = cov

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz

        self.pub.publish(msg)

        if self.do_publish_tf:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quaternion(self.yaw)
            self.tf_broadcaster.sendTransform(t)

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeWheelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
