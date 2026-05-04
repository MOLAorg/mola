#!/usr/bin/env python3
# Synthetic sensor publisher for MOLA state estimation demos and integration tests.
#
# Publishes noisy wheel odometry, optional visual/LiDAR odometry, IMU, and GNSS
# messages that match a closed-form ground-truth trajectory.  A ground_truth/pose
# topic is also published for external checkers.
#
# Trajectory options (parameter `scenario`):
#   static  :  robot stays still at (30 m E, 50 m N, yaw=60 deg)
#   moving  :  sinusoidal zigzag: x=v*t, y=A*sin(omega*t), v=1 m/s
#   circle  :  constant circular motion: vx=1 m/s, wz=0.2 rad/s
#
# Usage (standalone):
#   python3 fake_sensor_publisher.py --ros-args -p scenario:=circle
#   python3 fake_sensor_publisher.py --ros-args -p scenario:=moving -p seed:=42
#   python3 fake_sensor_publisher.py --ros-args -p scenario:=moving \
#       -p gnss_topic:=/gps -p odom2_topic:=/visual_odom
#
# Usage (installed):
#   python3 $(ros2 pkg prefix mola_demos)/share/mola_demos/demos/fake_sensor_publisher.py \
#       --ros-args -p scenario:=moving

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage

# ---------------------------------------------------------------------------
# Geodetic constants (Almería test origin, matching C++ unit tests)
# ---------------------------------------------------------------------------
_GT_LAT0_DEG = 36.8407
_GT_LON0_DEG = -2.4093
_GT_ALT0_M = 100.0
_M_PER_DEG_LAT = 111320.0

# Static scenario ground truth
_STATIC_X = 30.0
_STATIC_Y = 50.0
_STATIC_YAW = math.radians(60.0)

# Moving scenario parameters
_MOVING_V = 1.0        # forward speed [m/s]
_MOVING_A = 2.0        # lateral amplitude [m]
_MOVING_OMEGA = 0.5    # lateral frequency [rad/s]

# Circle scenario parameters
_CIRCLE_VX = 1.0       # forward speed [m/s]
_CIRCLE_WZ = 0.2       # yaw rate [rad/s]

_TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _quaternion_from_yaw(yaw):
    """Return (x, y, z, w) for a pure-yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def _quaternion_from_euler_zyx(roll, pitch, yaw):
    """ZYX Euler -> (x, y, z, w)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _imu_quat_from_enu_yaw(yaw_enu, pitch=0.0, roll=0.0):
    """
    Convert ENU yaw to IMU quaternion for imu_attitude_azimuth_offset_deg=0.

    The smoother recovers ENU yaw as: ENU_yaw = quat_yaw + pi/2.
    So we must publish: quat_yaw = ENU_yaw - pi/2.
    """
    return _quaternion_from_euler_zyx(roll, pitch, yaw_enu - math.pi / 2)


def _enu_to_latlon(east, north, alt,
                   lat0=_GT_LAT0_DEG, lon0=_GT_LON0_DEG, alt0=_GT_ALT0_M):
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return (lat0 + north / _M_PER_DEG_LAT,
            lon0 + east / max(m_per_deg_lon, 1e-9),
            alt0 + alt)


def _imu_linear_acc_body(yaw, pitch, roll, ax_world=0.0, ay_world=0.0):
    """Specific force in body frame (m/s²): (world_accel - gravity) rotated to body."""
    sf = np.array([ax_world, ay_world, 9.81]
                  )   # specific force in world (up = +Z)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    return R.T @ sf


# ---------------------------------------------------------------------------
# Ground-truth trajectories
# ---------------------------------------------------------------------------

def _gt_static(_t):
    return _STATIC_X, _STATIC_Y, 0.0, _STATIC_YAW, 0.0, 0.0


def _gt_static_twist(_t):
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def _gt_moving_pose(t):
    x = _MOVING_V * t
    y = _MOVING_A * math.sin(_MOVING_OMEGA * t)
    vy = _MOVING_A * _MOVING_OMEGA * math.cos(_MOVING_OMEGA * t)
    yaw = math.atan2(vy, _MOVING_V)
    return x, y, 0.0, yaw, 0.0, 0.0


def _gt_moving_twist(t):
    vx_w = _MOVING_V
    vy_w = _MOVING_A * _MOVING_OMEGA * math.cos(_MOVING_OMEGA * t)
    _, _, _, yaw, _, _ = _gt_moving_pose(t)
    vx = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)
    vy = -vx_w * math.sin(yaw) + vy_w * math.cos(yaw)
    denom = _MOVING_V ** 2 + vy_w ** 2
    wz = (-_MOVING_V * _MOVING_A * _MOVING_OMEGA ** 2 * math.sin(_MOVING_OMEGA * t)
          / max(denom, 1e-9))
    return vx, vy, 0.0, 0.0, 0.0, wz


def _gt_moving_world_accel(t):
    return 0.0, -_MOVING_A * _MOVING_OMEGA ** 2 * math.sin(_MOVING_OMEGA * t)


class _CircleIntegrator:
    """Integrates constant circular motion for the 'circle' scenario."""

    def __init__(self):
        self.x = self.y = self.yaw = 0.0

    def step(self, dt):
        self.x += _CIRCLE_VX * math.cos(self.yaw) * dt
        self.y += _CIRCLE_VX * math.sin(self.yaw) * dt
        self.yaw += _CIRCLE_WZ * dt

    def pose(self):
        return self.x, self.y, 0.0, self.yaw, 0.0, 0.0

    def twist(self):
        # Centripetal acceleration towards circle centre
        return _CIRCLE_VX, 0.0, 0.0, 0.0, 0.0, _CIRCLE_WZ

    def world_accel(self):
        # Centripetal: -v*wz in lateral world direction
        ac = _CIRCLE_VX * _CIRCLE_WZ
        return (-ac * math.sin(self.yaw), ac * math.cos(self.yaw))


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class FakeSensorPublisher(Node):

    def __init__(self):
        super().__init__('fake_sensor_publisher')

        # --- scenario ---
        self.declare_parameter('scenario', 'circle')
        self.declare_parameter('seed', 42)
        self.declare_parameter('startup_delay_sec', 3.0)

        # --- rates [Hz] ---
        self.declare_parameter('odom_rate', 50.0)
        self.declare_parameter('odom2_rate', 30.0)
        self.declare_parameter('gnss_rate', 2.0)
        self.declare_parameter('imu_rate', 100.0)

        # --- noise sigmas ---
        self.declare_parameter('gnss_xy_sigma_m', 1.0)
        self.declare_parameter('gnss_z_sigma_m', 2.0)
        self.declare_parameter('imu_yaw_sigma_rad', math.radians(2.0))
        self.declare_parameter('imu_acc_sigma', 0.05)
        self.declare_parameter('odom_lin_sigma', 0.02)
        self.declare_parameter('odom_ang_sigma', math.radians(0.5))
        self.declare_parameter('odom2_lin_sigma', 0.005)
        self.declare_parameter('odom2_ang_sigma', math.radians(0.1))
        self.declare_parameter('odom2_drift_y_per_step', 0.003)

        # --- topics (empty string = disabled) ---
        self.declare_parameter('odom_topic', '/wheel_odom')
        # visual/lidar odom, off by default
        self.declare_parameter('odom2_topic', '')
        self.declare_parameter('gnss_topic', '')         # off by default
        self.declare_parameter('imu_topic', '/imu')

        # --- frame names ---
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('gnss_frame', 'gnss_link')

        # -- read params --
        self._scenario = self.get_parameter('scenario').value
        self._rng = np.random.default_rng(self.get_parameter('seed').value)
        self._startup_delay = self.get_parameter('startup_delay_sec').value

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_link = self.get_parameter('base_link_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._gnss_frame = self.get_parameter('gnss_frame').value

        self._gnss_xy_sigma = self.get_parameter('gnss_xy_sigma_m').value
        self._gnss_z_sigma = self.get_parameter('gnss_z_sigma_m').value
        self._imu_yaw_sigma = self.get_parameter('imu_yaw_sigma_rad').value
        self._imu_acc_sigma = self.get_parameter('imu_acc_sigma').value
        self._odom_lin_sigma = self.get_parameter('odom_lin_sigma').value
        self._odom_ang_sigma = self.get_parameter('odom_ang_sigma').value
        self._odom2_lin_sigma = self.get_parameter('odom2_lin_sigma').value
        self._odom2_ang_sigma = self.get_parameter('odom2_ang_sigma').value
        self._odom2_drift_y = self.get_parameter(
            'odom2_drift_y_per_step').value

        odom_topic = self.get_parameter('odom_topic').value
        odom2_topic = self.get_parameter('odom2_topic').value
        gnss_topic = self.get_parameter('gnss_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        # -- publishers --
        self._pub_odom = (self.create_publisher(Odometry, odom_topic, 10)
                          if odom_topic else None)
        self._pub_odom2 = (self.create_publisher(Odometry, odom2_topic, 10)
                           if odom2_topic else None)
        self._pub_gnss = (self.create_publisher(NavSatFix, gnss_topic, 10)
                          if gnss_topic else None)
        self._pub_imu = (self.create_publisher(Imu, imu_topic, 10)
                         if imu_topic else None)
        self._pub_gt = self.create_publisher(
            PoseStamped, '/ground_truth/pose', 10)
        self._pub_tf_static = self.create_publisher(
            TFMessage, '/tf_static', _TRANSIENT_LOCAL_QOS)

        # -- trajectory state --
        self._odom_x = self._odom_y = self._odom_yaw = 0.0
        self._odom2_x = self._odom2_y = self._odom2_yaw = 0.0
        self._circle = _CircleIntegrator() if self._scenario == 'circle' else None

        self._t_node_start = None
        self._t_start = None

        # -- timers --
        if self._pub_odom:
            self.create_timer(
                1.0 / self.get_parameter('odom_rate').value, self._odom_cb)
        if self._pub_odom2:
            self.create_timer(
                1.0 / self.get_parameter('odom2_rate').value, self._odom2_cb)
        if self._pub_gnss:
            self.create_timer(
                1.0 / self.get_parameter('gnss_rate').value, self._gnss_cb)
        if self._pub_imu:
            self.create_timer(
                1.0 / self.get_parameter('imu_rate').value, self._imu_cb)
        self.create_timer(5.0, self._publish_tf_static)

        self._publish_tf_static()
        self.get_logger().info(
            f'fake_sensor_publisher: scenario={self._scenario}, seed={self.get_parameter("seed").value}, '
            f'startup_delay={self._startup_delay} s | '
            f'topics: odom={odom_topic!r} odom2={odom2_topic!r} '
            f'gnss={gnss_topic!r} imu={imu_topic!r}')

    # ------------------------------------------------------------------
    # Startup gating
    # ------------------------------------------------------------------

    def _maybe_start(self):
        now = time.monotonic()
        if self._t_node_start is None:
            self._t_node_start = now
        if self._t_start is None:
            if now - self._t_node_start < self._startup_delay:
                return False
            self._t_start = now
            self.get_logger().info('fake_sensor_publisher: starting publication')
        return True

    def _elapsed(self):
        return time.monotonic() - self._t_start

    # ------------------------------------------------------------------
    # Ground truth
    # ------------------------------------------------------------------

    def _gt_at(self, t):
        if self._scenario == 'static':
            return _gt_static(t)
        if self._scenario == 'moving':
            return _gt_moving_pose(t)
        return self._circle.pose()

    def _gt_twist_at(self, t):
        if self._scenario == 'static':
            return _gt_static_twist(t)
        if self._scenario == 'moving':
            return _gt_moving_twist(t)
        return self._circle.twist()

    def _gt_world_accel(self, t):
        if self._scenario == 'moving':
            return _gt_moving_world_accel(t)
        if self._scenario == 'circle':
            return self._circle.world_accel()
        return 0.0, 0.0

    # ------------------------------------------------------------------
    # Static TF
    # ------------------------------------------------------------------

    def _publish_tf_static(self):
        now = self.get_clock().now().to_msg()
        tfs = []
        for child in (self._imu_frame, self._gnss_frame):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._base_link
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            tfs.append(tf)
        self._pub_tf_static.publish(TFMessage(transforms=tfs))

    # ------------------------------------------------------------------
    # Odometry (wheel)
    # ------------------------------------------------------------------

    def _odom_cb(self):
        if not self._maybe_start():
            return
        t = self._elapsed()

        if self._scenario == 'circle':
            self._circle.step(1.0 / self.get_parameter('odom_rate').value)

        x, y, _z, yaw, _p, _r = self._gt_at(t)
        vx, _vy, _vz, _wx, _wy, wz = self._gt_twist_at(t)

        dt = 1.0 / self.get_parameter('odom_rate').value
        noisy_vx = vx + self._rng.normal(0, self._odom_lin_sigma)
        noisy_wz = wz + self._rng.normal(0, self._odom_ang_sigma)
        self._odom_x += noisy_vx * math.cos(self._odom_yaw) * dt
        self._odom_y += noisy_vx * math.sin(self._odom_yaw) * dt
        self._odom_yaw += noisy_wz * dt

        qx, qy, qz, qw = _quaternion_from_yaw(self._odom_yaw)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_link
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        sigma_pos = (self._odom_lin_sigma * max(t, 1.0)) ** 2 + 0.01
        sigma_ang = (self._odom_ang_sigma * max(t, 1.0)) ** 2 + 0.001
        msg.pose.covariance[0] = sigma_pos
        msg.pose.covariance[7] = sigma_pos
        msg.pose.covariance[35] = sigma_ang
        msg.twist.twist.linear.x = vx + \
            self._rng.normal(0, self._odom_lin_sigma)
        msg.twist.twist.angular.z = wz + \
            self._rng.normal(0, self._odom_ang_sigma)
        msg.twist.covariance[0] = self._odom_lin_sigma ** 2
        msg.twist.covariance[7] = self._odom_lin_sigma ** 2
        msg.twist.covariance[35] = self._odom_ang_sigma ** 2
        self._pub_odom.publish(msg)
        self._publish_gt(t, x, y, yaw)

    # ------------------------------------------------------------------
    # Odometry (visual / lidar :  lower noise, constant Y drift)
    # ------------------------------------------------------------------

    def _odom2_cb(self):
        if not self._maybe_start():
            return
        t = self._elapsed()
        x, y, _z, yaw, _p, _r = self._gt_at(t)
        vx, _vy, _vz, _wx, _wy, wz = self._gt_twist_at(t)

        dt = 1.0 / self.get_parameter('odom2_rate').value
        noisy_vx = vx + self._rng.normal(0, self._odom2_lin_sigma)
        noisy_wz = wz + self._rng.normal(0, self._odom2_ang_sigma)
        drift_y = self._odom2_drift_y
        self._odom2_x += (noisy_vx * math.cos(self._odom2_yaw)
                          - drift_y * math.sin(self._odom2_yaw)) * dt
        self._odom2_y += (noisy_vx * math.sin(self._odom2_yaw)
                          + drift_y * math.cos(self._odom2_yaw)) * dt
        self._odom2_yaw += noisy_wz * dt

        qx, qy, qz, qw = _quaternion_from_yaw(self._odom2_yaw)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom_visual'
        msg.child_frame_id = self._base_link
        msg.pose.pose.position.x = self._odom2_x
        msg.pose.pose.position.y = self._odom2_y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        sigma_pos = (self._odom2_lin_sigma * max(t, 1.0)) ** 2 + 0.001
        sigma_ang = (self._odom2_ang_sigma * max(t, 1.0)) ** 2 + 0.0001
        msg.pose.covariance[0] = sigma_pos
        msg.pose.covariance[7] = sigma_pos
        msg.pose.covariance[35] = sigma_ang
        msg.twist.twist.linear.x = vx + \
            self._rng.normal(0, self._odom2_lin_sigma)
        msg.twist.twist.angular.z = wz + \
            self._rng.normal(0, self._odom2_ang_sigma)
        msg.twist.covariance[0] = self._odom2_lin_sigma ** 2
        msg.twist.covariance[7] = self._odom2_lin_sigma ** 2
        msg.twist.covariance[35] = self._odom2_ang_sigma ** 2
        self._pub_odom2.publish(msg)

    # ------------------------------------------------------------------
    # GNSS
    # ------------------------------------------------------------------

    def _gnss_cb(self):
        if not self._maybe_start():
            return
        t = self._elapsed()
        x, y, z, _yaw, _p, _r = self._gt_at(t)

        lat, lon, alt = _enu_to_latlon(
            x + self._rng.normal(0, self._gnss_xy_sigma),
            y + self._rng.normal(0, self._gnss_xy_sigma),
            z + self._rng.normal(0, self._gnss_z_sigma),
        )
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._gnss_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = self._gnss_xy_sigma ** 2
        msg.position_covariance[4] = self._gnss_xy_sigma ** 2
        msg.position_covariance[8] = self._gnss_z_sigma ** 2
        self._pub_gnss.publish(msg)

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------

    def _imu_cb(self):
        if not self._maybe_start():
            return
        t = self._elapsed()
        x, y, z, yaw, pitch, roll = self._gt_at(t)
        _vx, _vy, _vz, _wx, _wy, wz = self._gt_twist_at(t)
        ax_w, ay_w = self._gt_world_accel(t)

        noisy_yaw = yaw + self._rng.normal(0, self._imu_yaw_sigma)
        qx, qy, qz, qw = _imu_quat_from_enu_yaw(noisy_yaw, pitch, roll)
        acc = _imu_linear_acc_body(yaw, pitch, roll, ax_w, ay_w)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        cov_ori = self._imu_yaw_sigma ** 2
        msg.orientation_covariance[0] = cov_ori
        msg.orientation_covariance[4] = cov_ori
        msg.orientation_covariance[8] = cov_ori
        gyro_sigma = math.radians(1.0)
        msg.angular_velocity.x = self._rng.normal(0, gyro_sigma)
        msg.angular_velocity.y = self._rng.normal(0, gyro_sigma)
        msg.angular_velocity.z = wz + self._rng.normal(0, gyro_sigma)
        msg.angular_velocity_covariance[0] = gyro_sigma ** 2
        msg.angular_velocity_covariance[4] = gyro_sigma ** 2
        msg.angular_velocity_covariance[8] = gyro_sigma ** 2
        msg.linear_acceleration.x = acc[0] + \
            self._rng.normal(0, self._imu_acc_sigma)
        msg.linear_acceleration.y = acc[1] + \
            self._rng.normal(0, self._imu_acc_sigma)
        msg.linear_acceleration.z = acc[2] + \
            self._rng.normal(0, self._imu_acc_sigma)
        cov_acc = self._imu_acc_sigma ** 2
        msg.linear_acceleration_covariance[0] = cov_acc
        msg.linear_acceleration_covariance[4] = cov_acc
        msg.linear_acceleration_covariance[8] = cov_acc
        self._pub_imu.publish(msg)

    # ------------------------------------------------------------------
    # Ground-truth pose publisher
    # ------------------------------------------------------------------

    def _publish_gt(self, t, x, y, yaw):
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'enu'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub_gt.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
