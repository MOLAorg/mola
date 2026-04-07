#!/usr/bin/env python3
"""
Publishes fake IMU data on /imu as sensor_msgs/Imu.

Simulates gravity-aligned accelerometer + gyroscope readings consistent
with the circular motion from the fake odometry publishers (vx=1 m/s, wz=0.2 rad/s).

Usage:
  python3 fake_imu_publisher.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import StaticTransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')

        self.declare_parameter('topic', '/imu')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('accel_noise', 0.05)    # m/s^2
        self.declare_parameter('gyro_noise', 0.005)    # rad/s
        self.declare_parameter('wz', 0.2)              # rad/s ground truth yaw rate
        self.declare_parameter('vx', 1.0)              # m/s ground truth forward speed
        self.declare_parameter('frame_id', 'imu_link')

        topic = self.get_parameter('topic').value
        rate = self.get_parameter('rate_hz').value
        self.accel_noise = self.get_parameter('accel_noise').value
        self.gyro_noise = self.get_parameter('gyro_noise').value
        self.wz = self.get_parameter('wz').value
        self.vx = self.get_parameter('vx').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Imu, topic, 10)
        self.dt = 1.0 / rate
        self.yaw = 0.0
        self.step = 0

        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Publish static transform base_link -> imu_link (identity)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_link'
        st.child_frame_id = self.frame_id
        st.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(st)

        self.get_logger().info(f'Publishing fake IMU on {topic} at {rate} Hz')

    def timer_callback(self):
        if self.step > 0:
            self.yaw += self.wz * self.dt

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation (gravity-aligned, yaw from integration)
        msg.orientation = yaw_to_quaternion(self.yaw)
        msg.orientation_covariance = [0.0] * 9
        msg.orientation_covariance[0] = 0.001
        msg.orientation_covariance[4] = 0.001
        msg.orientation_covariance[8] = 0.01

        # Angular velocity
        msg.angular_velocity.x = np.random.normal(0, self.gyro_noise)
        msg.angular_velocity.y = np.random.normal(0, self.gyro_noise)
        msg.angular_velocity.z = self.wz + np.random.normal(0, self.gyro_noise)
        msg.angular_velocity_covariance = [0.0] * 9
        msg.angular_velocity_covariance[0] = self.gyro_noise ** 2
        msg.angular_velocity_covariance[4] = self.gyro_noise ** 2
        msg.angular_velocity_covariance[8] = self.gyro_noise ** 2

        # Linear acceleration (centripetal + gravity)
        # In body frame: centripetal accel = v * w (pointing towards center = -y in body frame)
        centripetal = self.vx * self.wz
        msg.linear_acceleration.x = np.random.normal(0, self.accel_noise)
        msg.linear_acceleration.y = -centripetal + np.random.normal(0, self.accel_noise)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, self.accel_noise)  # gravity
        msg.linear_acceleration_covariance = [0.0] * 9
        msg.linear_acceleration_covariance[0] = self.accel_noise ** 2
        msg.linear_acceleration_covariance[4] = self.accel_noise ** 2
        msg.linear_acceleration_covariance[8] = self.accel_noise ** 2

        self.pub.publish(msg)
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
