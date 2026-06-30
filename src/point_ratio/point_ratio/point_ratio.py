import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float64

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import csv

# ── Sensor overlap constants ───────────────────────────────────────────────────
# Bottleneck per mode:
#   ouster128  → Oculii Eagle  : 120° az, ±15° el, 120 m
#   hesai/aeva → Hugin D1      : 100° az, ±15° el,  78 m
#   default    → Hugin D1      : 100° az, ±15° el,  78 m

FOV_PARAMS = {
    "ouster128": dict(max_range=120.0, h_fov=120.0, v_min=-15.0, v_max=15.0),
    "hesai":     dict(max_range=78.0,  h_fov=100.0, v_min=-15.0, v_max=15.0),
    "aeva":      dict(max_range=78.0,  h_fov=100.0, v_min=-15.0, v_max=15.0),
    "default":   dict(max_range=78.0,  h_fov=100.0, v_min=-15.0, v_max=15.0),
}


class PointRatio(Node):

    def __init__(self):
        super().__init__("point_ratio")

        # ---------------- state ----------------
        self.radar_points = 1.0
        self.lidar_points = 1.0
        self.max_points   = 1.0

        self.lidar_xyz = np.zeros((0, 3))
        self.radar_xyz = np.zeros((0, 3))

        self.log        = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self._lidar_ratio  = 1.0
        self._sensor_ratio = 1.0

        # ---------------- param ----------------
        self.mode = self.declare_parameter("mode", "ouster128").value

        p = FOV_PARAMS.get(self.mode, FOV_PARAMS["default"])
        self.max_range = p["max_range"]
        self.h_fov     = p["h_fov"]
        self.v_min     = p["v_min"]
        self.v_max     = p["v_max"]

        # ---------------- publishers ----------------
        self.pub       = self.create_publisher(Float64, "point_ratio",  10)
        self.pub_lidar = self.create_publisher(Float64, "lidar_points", 10)
        self.pub_radar = self.create_publisher(Float64, "radar_points", 10)

        # ---------------- subscribers ----------------
        if self.mode == "livox":
            self.create_subscription(PointCloud, "/radar_enhanced_pcl", self.radar_callback, 10)

        elif self.mode == "ouster128":
            self.create_subscription(PointCloud2, "/ouster/points",            self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/oculii_radar/point_cloud", self.radar_callback, 10)

        elif self.mode == "hesai":
            self.create_subscription(PointCloud2, "/hesai/points",               self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.radar_callback, 10)

        elif self.mode == "aeva":
            self.create_subscription(PointCloud2, "/aeva/ATLAS/point_cloud_compensated", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data",         self.radar_callback, 10)

        else:
            self.create_subscription(PointCloud2, "/hugin_raf_1/radar_data", self.radar_callback, 10)
            self.create_subscription(PointCloud2, "/ouster/points",          self.lidar_callback, 10)

    # ── FOV filter ────────────────────────────────────────────────────────────
    def filter_by_fov_and_range(self, pts):
        if len(pts) == 0:
            return np.zeros((0, 3))

        ranges    = np.linalg.norm(pts, axis=1)
        mask      = (ranges > 0.5) & (ranges < self.max_range)

        xy_ranges = np.linalg.norm(pts[:, :2], axis=1)
        elevation = np.degrees(np.arctan2(pts[:, 2], xy_ranges))
        mask     &= (elevation >= self.v_min) & (elevation <= self.v_max)

        azimuth   = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        mask     &= (azimuth >= -self.h_fov / 2) & (azimuth <= self.h_fov / 2)

        return pts[mask]

    def cloud2_to_xyz(self, msg):
        pts = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(pts) == 0:
            return np.zeros((0, 3))
        return pts[~np.all(pts == 0, axis=1)]

    # ── probability ───────────────────────────────────────────────────────────
    def probability(self):
        # half 1: reflective lidar points vs total scan points
        lidar_ratio = self.lidar_points / max(self.max_points, 1.0)

        # half 2: FOV-filtered lidar vs radar point count
        lidar_filtered = self.filter_by_fov_and_range(self.lidar_xyz)
        radar_filtered = self.filter_by_fov_and_range(self.radar_xyz)
        fov_ratio = len(lidar_filtered) / max(len(radar_filtered), 1)
        fov_ratio = min(fov_ratio, 1.0)

        # final probability
        self._sensor_ratio = 0.5 * lidar_ratio + 0.5 * fov_ratio

        self.publish_values()

        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.log.append([t, self._sensor_ratio])

    # ── callbacks ─────────────────────────────────────────────────────────────
    def radar_callback(self, msg):
        if self.mode == "livox":
            self.radar_points = float(len(msg.points))
            pts = [[p.x, p.y, p.z] for p in msg.points]
            self.radar_xyz = np.array(pts) if pts else np.zeros((0, 3))
        else:
            self.radar_points = float(msg.width * msg.height)
            self.radar_xyz    = self.cloud2_to_xyz(msg)

        self.probability()

    def lidar_callback(self, msg):
        if self.mode == "livox":
            self.lidar_points = 1.0
            pts = []
            for p in msg.points:
                if p.reflectivity > 0:
                    self.lidar_points += 1
                pts.append([p.x, p.y, p.z])
            self.max_points = float(msg.point_num)
            self.lidar_xyz  = np.array(pts) if pts else np.zeros((0, 3))
        else:
            self.lidar_xyz    = self.cloud2_to_xyz(msg)
            self.lidar_points = float(len(self.lidar_xyz))
            self.max_points   = float(msg.width * msg.height)

        self.probability()

    # ── publish ───────────────────────────────────────────────────────────────
    def publish_values(self):
        self.pub.publish(Float64(data=float(self._sensor_ratio)))
        self.pub_lidar.publish(Float64(data=float(self.lidar_points)))
        self.pub_radar.publish(Float64(data=float(self.radar_points)))

    # ── save ──────────────────────────────────────────────────────────────────
    def save_csv(self):
        with open("visibility_log.csv", "w") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "prob"])
            writer.writerows(self.log)


def main(args=None):
    rclpy.init(args=args)
    node = PointRatio()
    try:
        rclpy.spin(node)
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()