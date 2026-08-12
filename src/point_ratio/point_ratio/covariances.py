#!/usr/bin/env python3
import bisect
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class SmokeGatedCovarianceNode(Node):
    def __init__(self):
        super().__init__('smoke_gated_covariance_node')

        self.declare_parameter('lidar_odom_topic', '/genz/odometry')
        self.declare_parameter('radar_odom_topic', '/kiss/odometry')
        self.declare_parameter('smoke_topic', '/point_ratio')
        self.declare_parameter('lidar_odom_out_topic', '/odom/lidar')
        self.declare_parameter('radar_odom_out_topic', '/odom/radar')

        self.declare_parameter('lidar_trust_min', 0.001)
        self.declare_parameter('radar_trust_min', 0.001)

        self.smoke_prob = 1.0  # assume good lidar until told otherwise
        self.lidar_on=True
        self.last_smoke_stamp = None

        gp = self.get_parameter
        self.lidar_topic = gp('lidar_odom_topic').value
        self.radar_topic = gp('radar_odom_topic').value
        self.smoke_topic = gp('smoke_topic').value
        self.lidar_out = gp('lidar_odom_out_topic').value
        self.radar_out = gp('radar_odom_out_topic').value
        self.lidar_trust_min = float(gp('lidar_trust_min').value)
        self.radar_trust_min = float(gp('radar_trust_min').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Float32, self.smoke_topic, self._smoke_cb, qos)
        self.create_subscription(Odometry, self.lidar_topic, self._lidar_cb, qos)
        self.create_subscription(Odometry, self.radar_topic, self._radar_cb, qos)

        self.pub_lidar = self.create_publisher(Odometry, self.lidar_out, qos)
        self.pub_radar = self.create_publisher(Odometry, self.radar_out, qos)

    def _smoke_cb(self, msg: Float32):
        self.smoke_prob = float(np.clip(msg.data, 0.0, 1.0))
        self.last_smoke_stamp = self.get_clock().now()

    def _lidar_trust(self, p):
        return self.lidar_trust_min + (1.0 - self.lidar_trust_min) * p

    def _radar_trust(self, p):
        return self.radar_trust_min + (1.0 - self.radar_trust_min) * (1.0 - p)

    def _lidar_cb(self, msg: Odometry):
        if self.smoke_prob < 0.2:
            # severe degeneracy: don't fuse lidar at all this cycle
            self.lidar_on=False
            return
        
        if self.smoke_prob > 0.7 and self.lidar_on==False:
            self.lidar_on=True

        if self.lidar_on:
            trust = self._lidar_trust(self.smoke_prob)
            msg.pose.covariance = tuple(c / trust for c in msg.pose.covariance)
            msg.twist.covariance = tuple(c / trust for c in msg.twist.covariance)
            self.pub_lidar.publish(msg)

    def _radar_cb(self, msg: Odometry):
        trust = self._radar_trust(self.smoke_prob)
        msg.pose.covariance = tuple(c / trust for c in msg.pose.covariance)
        msg.twist.covariance = tuple(c / trust for c in msg.twist.covariance)
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