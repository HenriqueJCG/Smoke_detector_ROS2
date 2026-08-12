import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float64
from scipy.spatial import cKDTree


class PointRatio(Node):

    def __init__(self):
        super().__init__("point_ratio")

        # Initialize parameters
        self.match_threshold = self.declare_parameter("match_threshold", 1.0).value
        self.alpha = self.declare_parameter("smoothing", 0.1).value
        self.mode = self.declare_parameter("mode", "hesai").value
        self.running = True

        # Initialize variables
        self.radar_points = 1.0
        self.lidar_points = 1.0
        self.max_points = 1.0

        self.lidar_xyz = None
        self.kdtree = None
        self.kdtree_dirty = False


        # Export data to CSV file
        self.log = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9


        # ROS publishers
        self.pub_prob = self.create_publisher(Float64, "point_ratio", 10)
        self.pub_lidar = self.create_publisher(Float64, "lidar_points", 10)
        self.pub_radar = self.create_publisher(Float64, "radar_points", 10)

        # ROS subscribers
        if self.mode == "hesai":
            self.create_subscription(PointCloud2, "/hesai/points",               self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.radar_callback, 10)
            self.pub_lidar_pcl = self.create_publisher(PointCloud2, "/fitered_hesai", 10)
        
        elif self.mode == "aeva":
            self.create_subscription(PointCloud2, "/aeva/ATLAS/point_cloud_compensated", self.lidar_callback, 10)
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data",         self.radar_callback, 10)
            self.pub_lidar_pcl = self.create_publisher(PointCloud2, "/filtered_aeva", 10)
        
    # Convert pointcloud message to numpy array of xyz coordinates
    def cloud_to_xyz(self, msg):
        pts = []
        #Filter out nan points
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #Filter out points other irrelevant points (0,0,0)
            if not (p[0] == 0 and p[1] == 0 and p[2] == 0):
                pts.append([p[0], p[1], p[2]])
        return np.array(pts)

    def publish(self, prob):

        self.pub_prob.publish(Float64(data=float(prob)))
        self.pub_lidar.publish(Float64(data=float(self.lidar_points)))
        self.pub_radar.publish(Float64(data=float(self.radar_points)))

    def lidar_callback(self, msg):

        pts = []

        #Update LiDAR points count
        self.lidar_points = sum(1 for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True) if not (p[0] == 0 and p[1] == 0 and p[2] == 0))

        self.max_points = float(msg.width * msg.height)
        pts = self.cloud_to_xyz(msg) #Create array of xyz coordinates from pointcloud message

        if len(pts) == 0:
            return

        self.lidar_xyz = pts

        #If there are enough LiDAR points, create a KD-tree for fast nearest neighbor search
        if len(self.lidar_xyz) > 10:
            self.kdtree = cKDTree(self.lidar_xyz)
            self.kdtree_dirty = False

        if self.prob<0.2:
           self.running=False
        elif self.prob>0.7 and not self.running:
            self.running=True

        if self.running:
                self.pub_lidar_pcl.publish(msg)

    def radar_callback(self, msg):

        #See if KD-tree is available, if not, return
        if self.kdtree is None:
            return

        # Get radar xyz coordinates array from pointcloud message and total radar points count
        pts = self.cloud_to_xyz(msg)
        self.radar_points = float(msg.width * msg.height)

        if len(pts) == 0:
            return

        #Convert to numpy array for KD-tree query
        pts = np.array(pts)

        # Query KD-tree for nearest neighbor distances and count matches within threshold
        dists, _ = self.kdtree.query(pts, k=1)
        matches = np.sum(dists < self.match_threshold)

        #The tree match ratio will be the number of matches divided by the total number of radar points
        match_ratio = matches / float(len(pts))

        #Calculate total LiDAR ratio
        lidar_ratio = self.lidar_points / max(self.max_points, 1.0)

        #Calculate final probability as the average of the match ratio and the LiDAR ratio
        self.prob = 0.5 * match_ratio + 0.5 * lidar_ratio

        self.publish(self.prob)

        # Append time and probability to log for CSV export
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.log.append([t, self.prob])


    # Export prob data to csv file
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