#!/usr/bin/env python3
"""
Publishes fake visual odometry on /visual_odom as nav_msgs/Odometry.

Simulates a robot driving in a circle at 1 m/s with 0.2 rad/s yaw rate
(same ground truth as the wheel odom publisher), but with different noise
characteristics: lower white noise but a constant sideways (Y) drift.

Usage:
  ros2 run mola_state_estimation_smoother fake_visual_odom_publisher.py
  # or directly:
  python3 fake_visual_odom_publisher.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class FakeVisualOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_visual_odom_publisher')

        # Parameters
        self.declare_parameter('topic', '/visual_odom')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('noise_xyz', 0.005)      # m per step (more precise)
        self.declare_parameter('noise_ang', 0.001)       # rad per step
        self.declare_parameter('drift_y_per_step', 0.003)  # constant lateral drift
        self.declare_parameter('vx', 1.0)                # m/s ground truth
        self.declare_parameter('wz', 0.2)                # rad/s ground truth
        self.declare_parameter('frame_id', 'odom_visual')
        self.declare_parameter('child_frame_id', 'base_link')

        topic = self.get_parameter('topic').value
        rate = self.get_parameter('rate_hz').value
        self.noise_xyz = self.get_parameter('noise_xyz').value
        self.noise_ang = self.get_parameter('noise_ang').value
        self.drift_y = self.get_parameter('drift_y_per_step').value
        self.vx = self.get_parameter('vx').value
        self.wz = self.get_parameter('wz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.pub = self.create_publisher(Odometry, topic, 10)

        self.dt = 1.0 / rate
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.step = 0

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(
            f'Publishing fake visual odom on {topic} at {rate} Hz '
            f'(noise_xyz={self.noise_xyz}, drift_y={self.drift_y})')

    def timer_callback(self):
        if self.step > 0:
            # Ground truth delta
            dx_gt = self.vx * self.dt
            dyaw_gt = self.wz * self.dt

            # Visual odom: low noise but systematic Y drift
            dx = dx_gt + np.random.normal(0, self.noise_xyz)
            dy = self.drift_y + np.random.normal(0, self.noise_xyz)
            dz = np.random.normal(0, self.noise_xyz)
            dyaw = dyaw_gt + np.random.normal(0, self.noise_ang)

            # Integrate in local frame
            self.x += dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
            self.y += dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            self.z += dz
            self.yaw += dyaw

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        # Covariance (diagonal, tighter than wheel odom)
        cov = [0.0] * 36
        cov[0] = self.noise_xyz ** 2
        cov[7] = self.noise_xyz ** 2
        cov[14] = self.noise_xyz ** 2
        cov[35] = self.noise_ang ** 2
        msg.pose.covariance = cov

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz

        self.pub.publish(msg)
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisualOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
