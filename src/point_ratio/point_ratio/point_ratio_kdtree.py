import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float64


import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree
import numpy as np
import csv


class PointRatio(Node):

    def __init__(self):
        super().__init__("point_ratio")

        # ---------------- params ----------------
        self.match_threshold = self.declare_parameter(
            "match_threshold", 1.0
        ).value

        self.alpha = self.declare_parameter(
            "smoothing", 0.1
        ).value

        self.mode = self.declare_parameter(
            "mode", "ouster128"
        ).value

        # ---------------- state ----------------
        self.radar_points = 1.0
        self.lidar_points = 1.0
        self.max_points = 1.0

        self.lidar_xyz = None
        self.kdtree = None
        self.kdtree_dirty = False

        self.log = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # ---------------- ROS I/O ----------------
        self.pub = self.create_publisher(Float64, "point_ratio", 10)
        self.pub_lidar = self.create_publisher(Float64, "lidar_points", 10)
        self.pub_radar = self.create_publisher(Float64, "radar_points", 10)

        # ---------------- subscribers ----------------
        if self.mode == "livox":
            self.create_subscription(PointCloud, "/radar_enhanced_pcl", self.radar_callback, 10)
            #self.create_subscription(CustomMsg, "/livox/lidar", self.lidar_callback, 10)

        elif self.mode == "ouster128":
            self.create_subscription(PointCloud2, "/ouster/points", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/oculii_radar/point_cloud", self.radar_callback, 10)
        
        elif self.mode == "hesai":
            self.create_subscription(PointCloud2, "/hesai/points", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.radar_callback, 10)

        elif self.mode == "aeva":    
            self.create_subscription(PointCloud2, "/aeva/ATLAS/point_cloud_compensated", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.radar_callback, 10)

        else:
            self.create_subscription(PointCloud2, "/hugin_raf_1/radar_data", self.radar_callback, 10)
            self.create_subscription(PointCloud2, "/ouster/points", self.lidar_callback, 10)

    # ---------------- utils ----------------
    def cloud_to_xyz(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if not (p[0] == 0 and p[1] == 0 and p[2] == 0):
                pts.append([p[0], p[1], p[2]])
        return np.array(pts)

    def publish(self, prob):
        msg = Float64()
        msg.data = float(prob)

        self.pub.publish(msg)
        self.pub_lidar.publish(Float64(data=float(self.lidar_points)))
        self.pub_radar.publish(Float64(data=float(self.radar_points)))

    # ---------------- callbacks ----------------
    def lidar_callback(self, msg):

        pts = []

        if self.mode == "livox":
            self.lidar_points = 1.0
            for p in msg.points:
                if p.reflectivity > 0:
                    self.lidar_points += 1
                pts.append([p.x, p.y, p.z])

            self.max_points = float(msg.point_num)

        else:
            self.lidar_points = sum(
                1 for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                if not (p[0] == 0 and p[1] == 0 and p[2] == 0)
            )

            self.max_points = float(msg.width * msg.height)
            pts = self.cloud_to_xyz(msg)

        if len(pts) == 0:
            return

        self.lidar_xyz = pts

        if len(self.lidar_xyz) > 10:
            self.kdtree = cKDTree(self.lidar_xyz)
            self.kdtree_dirty = False

    def radar_callback(self, msg):

        if self.kdtree is None:
            return

        # -------- radar points --------
        if self.mode == "livox":
            pts = [[p.x, p.y, p.z] for p in msg.points]
            self.radar_points = float(len(msg.points))
        else:
            pts = self.cloud_to_xyz(msg)
            self.radar_points = float(msg.width * msg.height)

        if len(pts) == 0:
            return

        pts = np.array(pts)

        # -------- KD-tree matching --------
        dists, _ = self.kdtree.query(pts, k=1)
        matches = np.sum(dists < self.match_threshold)

        match_ratio = matches / float(len(pts))

        lidar_ratio = self.lidar_points / max(self.max_points, 1.0)

        prob = 0.5 * match_ratio + 0.5 * lidar_ratio

        # -------- publish --------
        self.publish(prob)

        # -------- logging --------
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.log.append([t, prob])

    # ---------------- save ----------------
    def save_csv(self):
        with open("visibility_log_kdtree.csv", "w") as f:
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