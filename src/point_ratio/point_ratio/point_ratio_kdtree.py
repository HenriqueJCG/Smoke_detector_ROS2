import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Float64
from scipy.spatial import cKDTree
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


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
        self.prob=1


        # Export data to CSV file
        self.log = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.base_frame = "base_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS publishers
        self.pub_prob = self.create_publisher(Float64, "point_ratio", 10)
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
        

    def transform_points_to_base(self, pts, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            # Rotation matrix from quaternion
            x, y, z, w = q.x, q.y, q.z, q.w

            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
                [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
            ])

            translation = np.array([t.x, t.y, t.z])

            return pts @ R.T + translation

        except Exception as e:
            self.get_logger().warning(
                f"Could not transform {msg.header.frame_id} -> "
                f"{self.base_frame}: {e}"
            )
            return None

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

        self.lidar_xyz = self.cloud_to_xyz(msg)

        if len(self.lidar_xyz) == 0:
            return

        self.lidar_xyz = self.transform_points_to_base(
            self.lidar_xyz, msg
        )

        if self.lidar_xyz is None:
            return

        #Update LiDAR points count
        self.lidar_points = sum(1 for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True) if not (p[0] == 0 and p[1] == 0 and p[2] == 0))

        self.max_points = float(msg.width * msg.height)
        

        #If there are enough LiDAR points, create a KD-tree for fast nearest neighbor search
        if len(self.lidar_xyz) > 10:
            self.kdtree = cKDTree(self.lidar_xyz)
            self.kdtree_dirty = False

        if self.prob<0.15:
           self.running=False
        elif self.prob>0.4 and not self.running:
            self.running=True

        if self.running:
                self.pub_lidar_pcl.publish(msg)

    def radar_callback(self, msg):

        #See if KD-tree is available, if not, return
        if self.kdtree is None:
            return

        self.radar_xyz = self.cloud_to_xyz(msg)

        if len(self.radar_xyz) == 0:
            return

        self.radar_xyz = self.transform_points_to_base(
            self.radar_xyz, msg
        )

        if self.radar_xyz is None:
            return

        self.radar_points = float(len(self.radar_xyz))


        #Convert to numpy array for KD-tree query
        pts = np.array(self.radar_xyz)

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
        if not self.running:
             self.pub_radar_pcl.publish(msg)

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