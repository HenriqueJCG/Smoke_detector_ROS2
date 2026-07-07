#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def _apply_baseline(cov_flat, baseline_diag):
    cov = np.array(cov_flat, dtype=float).reshape(6, 6)
    if np.allclose(np.diag(cov), 0.0, atol=1e-12):
        cov = np.diag(baseline_diag).astype(float)
    return cov


def _scale(cov6x6, factor):
    return (cov6x6 * factor).flatten().tolist()


class SmokeGatedCovarianceNode(Node):
    def __init__(self):
        super().__init__('smoke_gated_covariance_node')

        # ---- topics ----
        self.declare_parameter('lidar_odom_topic', '/genz/odometry')
        self.declare_parameter('radar_odom_topic', '/kiss/odometry')
        self.declare_parameter('smoke_topic', '/point_ratio')
        self.declare_parameter('lidar_odom_out_topic', '/odom/lidar')
        self.declare_parameter('radar_odom_out_topic', '/odom/radar')

        # ---- trust model ----
        # worst-case trust (0 < trust_min <= 1) each sensor can fall to
        self.declare_parameter('lidar_trust_min', 0.05)
        self.declare_parameter('radar_trust_min', 0.05)
        # exponent applied to smoke probability before mapping to trust;
        # 1.0 = linear, >1.0 = only reacts once smoke prob is quite high,
        # <1.0 = reacts early/aggressively
        self.declare_parameter('smoke_gain', 1.0)
        # flip which sensor is favored when smoke probability is high
        self.declare_parameter('invert', True)
        # if no smoke message received within this window, fall back to
        # neutral trust (1.0, 1.0) rather than guessing
        self.declare_parameter('smoke_timeout_sec', 1.0)

        # ---- baseline covariance diagonals, used only if the raw message
        # covariance is all-zero (see module docstring) ----
        # order: x, y, z, roll, pitch, yaw
        self.declare_parameter('lidar_baseline_pose_diag',
                                [0.01, 0.01, 0.05, 0.02, 0.02, 0.02])
        self.declare_parameter('lidar_baseline_twist_diag',
                                [0.02, 0.02, 0.05, 0.02, 0.02, 0.02])
        self.declare_parameter('radar_baseline_pose_diag',
                                [0.05, 0.05, 0.20, 0.10, 0.10, 0.05])
        self.declare_parameter('radar_baseline_twist_diag',
                                [0.05, 0.05, 0.20, 0.10, 0.10, 0.05])

        gp = self.get_parameter
        self.lidar_topic = gp('lidar_odom_topic').value
        self.radar_topic = gp('radar_odom_topic').value
        self.smoke_topic = gp('smoke_topic').value
        self.lidar_out = gp('lidar_odom_out_topic').value
        self.radar_out = gp('radar_odom_out_topic').value
        self.lidar_trust_min = float(gp('lidar_trust_min').value)
        self.radar_trust_min = float(gp('radar_trust_min').value)
        self.gain = float(gp('smoke_gain').value)
        self.invert = bool(gp('invert').value)
        self.smoke_timeout = float(gp('smoke_timeout_sec').value)
        self.lidar_base_pose = list(gp('lidar_baseline_pose_diag').value)
        self.lidar_base_twist = list(gp('lidar_baseline_twist_diag').value)
        self.radar_base_pose = list(gp('radar_baseline_pose_diag').value)
        self.radar_base_twist = list(gp('radar_baseline_twist_diag').value)

        self.smoke_prob = 0.0
        self.last_smoke_stamp = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Float32, self.smoke_topic, self._smoke_cb, qos)
        self.create_subscription(Odometry, self.lidar_topic, self._lidar_cb, qos)
        self.create_subscription(Odometry, self.radar_topic, self._radar_cb, qos)

        self.pub_lidar = self.create_publisher(Odometry, self.lidar_out, qos)
        self.pub_radar = self.create_publisher(Odometry, self.radar_out, qos)

        self.get_logger().info(
            f'smoke_gated_covariance_node up: '
            f'lidar {self.lidar_topic} -> {self.lidar_out}, '
            f'radar {self.radar_topic} -> {self.radar_out}, '
            f'smoke <- {self.smoke_topic}, invert={self.invert}'
        )

    def _smoke_cb(self, msg: Float32):
        self.smoke_prob = float(np.clip(msg.data, 0.0, 1.0))
        self.last_smoke_stamp = self.get_clock().now()

    def _stale(self):
        if self.last_smoke_stamp is None:
            return True
        age = (self.get_clock().now() - self.last_smoke_stamp).nanoseconds * 1e-9
        return age > self.smoke_timeout

    def _trusts(self):
        if self._stale():
            return 1.0, 1.0  # no signal -> don't touch anything

        p = self.smoke_prob ** self.gain

        # literal request: high smoke -> trust lidar more, radar less.
        # set `invert:=true` to swap (usually the physically sensible choice
        # for optical lidar + smoke/dust).
        lidar_trust = self.lidar_trust_min + (1.0 - self.lidar_trust_min) * p
        radar_trust = self.radar_trust_min + (1.0 - self.radar_trust_min) * (1.0 - p)

        if self.invert:
            lidar_trust, radar_trust = radar_trust, lidar_trust

        return lidar_trust, radar_trust

    def _lidar_cb(self, msg: Odometry):
        lidar_trust, _ = self._trusts()
        pose_cov = _apply_baseline(msg.pose.covariance, self.lidar_base_pose)
        twist_cov = _apply_baseline(msg.twist.covariance, self.lidar_base_twist)
        msg.pose.covariance = _scale(pose_cov, 1.0 / lidar_trust)
        msg.twist.covariance = _scale(twist_cov, 1.0 / lidar_trust)
        self.pub_lidar.publish(msg)

    def _radar_cb(self, msg: Odometry):
        _, radar_trust = self._trusts()
        pose_cov = _apply_baseline(msg.pose.covariance, self.radar_base_pose)
        twist_cov = _apply_baseline(msg.twist.covariance, self.radar_base_twist)
        msg.pose.covariance = _scale(pose_cov, 1.0 / radar_trust)
        msg.twist.covariance = _scale(twist_cov, 1.0 / radar_trust)
        self.pub_radar.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SmokeGatedCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()