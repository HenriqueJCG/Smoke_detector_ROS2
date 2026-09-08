import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float64

FOV_PARAMS = {
    "hesai":     dict(max_range=20.0,  h_fov=100.0, v_min=-15.0, v_max=15.0),
    "aeva":      dict(max_range=78.0,  h_fov=100.0, v_min=-15.0, v_max=15.0),
}


class PointRatio(Node):

    def __init__(self):
        super().__init__("point_ratio")

        # Initialize variables
        self.radar_points = 1.0
        self.lidar_points = 1.0
        self.max_points   = 1.0
        self.running = True

        self.lidar_xyz = np.zeros((0, 3))
        self.radar_xyz = np.zeros((0, 3))

        self.log        = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self._lidar_ratio  = 1.0
        self._sensor_ratio = 1.0

        # Parameters
        self.mode = self.declare_parameter("mode", "hesai").value

        p = FOV_PARAMS.get(self.mode, FOV_PARAMS["hesai"])
        self.max_range = p["max_range"]
        self.h_fov     = p["h_fov"]
        self.v_min     = p["v_min"]
        self.v_max     = p["v_max"]

        # ROS publishers 
        self.pub       = self.create_publisher(Float64, "point_ratio",  10)
        self.pub_lidar = self.create_publisher(Float64, "lidar_points", 10)
        self.pub_radar = self.create_publisher(Float64, "radar_points", 10)
        self.pub_radar_pcl = self.create_publisher(PointCloud2, "/filtered_hugin", 10)

        # ROS subscribers
        if self.mode == "hesai":
            self.create_subscription(PointCloud2, "/hesai/points",               self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.radar_callback, 10)
            self.pub_lidar_pcl = self.create_publisher(PointCloud2, "/filtered_hesai", 10)
        
        elif self.mode == "aeva":
            self.create_subscription(PointCloud2, "/aeva/ATLAS/point_cloud_compensated", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data",         self.radar_callback, 10)
            self.pub_lidar_pcl = self.create_publisher(PointCloud2, "/filtered_aeva", 10)
        
    # FOV filter
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

    # Convert pointcloud message to numpy array of xyz coordinates
    def cloud_to_xyz(self, msg):
        pts = []
        #Filter out nan points
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #Filter out points other irrelevant points (0,0,0)
            if not (p[0] == 0 and p[1] == 0 and p[2] == 0):
                pts.append([p[0], p[1], p[2]])
        return np.array(pts)

    def probability(self):
        
        # FOV-filtered lidar vs radar point count
        lidar_filtered = self.filter_by_fov_and_range(self.lidar_xyz)
        radar_filtered = self.filter_by_fov_and_range(self.radar_xyz)
        fov_ratio = len(lidar_filtered) / max(len(radar_filtered), 1)
        fov_ratio = min(fov_ratio, 1.0)

        #Calculate total LiDAR ratio
        self._lidar_ratio = self.lidar_points / max(self.max_points, 1.0)

        # 
        self._sensor_ratio = 0.5 * self._lidar_ratio + 0.5 * fov_ratio

        if self._sensor_ratio<0.2:
            self.running=False
        elif self._sensor_ratio>0.7 and not self.running:
            self.running=True

        self.publish()

        # Append time and probability to log for CSV export
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.log.append([t, self._sensor_ratio])

    def radar_callback(self, msg):

        self.radar_points = float(msg.width * msg.height)
        self.radar_xyz    = self.cloud_to_xyz(msg)

        self.probability()
        if not self.running:
             self.pub_radar_pcl.publish(msg)

    def lidar_callback(self, msg):
        pts = []

        self.lidar_xyz    = self.cloud_to_xyz(msg)
        self.lidar_points = float(len(self.lidar_xyz))
        self.max_points   = float(msg.width * msg.height)

        self.probability()
        if self.running:
             self.pub_lidar_pcl.publish(msg)

    
    def publish(self):
        self.pub.publish(Float64(data=float(self._sensor_ratio)))
        self.pub_lidar.publish(Float64(data=float(self.lidar_points)))
        self.pub_radar.publish(Float64(data=float(self.radar_points)))

    # Export prob data to csv file
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