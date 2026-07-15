#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class RadarEgoVelocity(Node):

    def __init__(self):
        super().__init__("radar_ego_velocity")

        self.declare_parameter("doppler_field", "doppler") #velocity
        self.declare_parameter("min_points", 10)
        self.declare_parameter("ransac_iterations", 50)
        self.declare_parameter("inlier_threshold", 0.15)  
        self.declare_parameter("publish_filtered_cloud", False)
        self.declare_parameter("integrate_position", True)
        self.declare_parameter("mode", "hugin")

        self.doppler_field = self.get_parameter("doppler_field").value
        self.min_points = self.get_parameter("min_points").value
        self.ransac_iterations = self.get_parameter("ransac_iterations").value
        self.inlier_threshold = self.get_parameter("inlier_threshold").value
        self.publish_filtered_cloud = self.get_parameter("publish_filtered_cloud").value
        self.integrate_position = self.get_parameter("integrate_position").value
        self.mode = self.get_parameter("mode").value

        self.last_imu_quat = None  
        self.last_time = None
        self.position = np.zeros(3)

        if self.mode == "aeva":
            self.create_subscription(PointCloud2, "/aeva/ATLAS/point_cloud_compensated", self.cloud_callback, 1)
        else:
            self.create_subscription(PointCloud2, "/sensrad/radar_1/radar_data", self.cloud_callback,1)

        if self.integrate_position:
            self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, "/radar/ego_velocity", 10)
        
        if self.integrate_position:
            self.odom_pub = self.create_publisher(Odometry, "/radar/odometry_dead_reckoned", 10)
        if self.publish_filtered_cloud:
            self.filtered_pub = self.create_publisher(PointCloud2, "/radar/dynamic_removed", 1)

    def imu_callback(self, msg):
        q = msg.orientation
        self.last_imu_quat = (q.x, q.y, q.z, q.w)

    def cloud_callback(self, msg):
        points = pc2.read_points(
            msg, field_names=("x", "y", "z", self.doppler_field), skip_nans=True
        )
        
        if len(points) < self.min_points:
            self.get_logger().warn(
                f"Radar ego-velocity: not enough points ({len(points)}) this scan",
                throttle_duration_sec=5.0,
            )
            return

        xyz = np.stack(
            [points["x"], points["y"], points["z"]], axis=-1
        ).astype(np.float64)
        doppler = np.asarray(points[self.doppler_field], dtype=np.float64)

        ranges = np.linalg.norm(xyz, axis=1)
        valid = ranges > 1e-3 
        xyz, doppler, ranges = xyz[valid], doppler[valid], ranges[valid]
        if len(xyz) < self.min_points:
            return

        directions = xyz / ranges[:, None] 

        velocity, inlier_mask = self.ransac_ego_velocity(directions, doppler)
        if velocity is not None:
            velocity = np.array([
                velocity[1],    
                -velocity[0],   
                velocity[2],
            ])

        self.publish_twist(velocity, msg.header)

        if self.integrate_position:
            self.integrate_and_publish(velocity, msg.header)

        if self.publish_filtered_cloud:
            self.publish_filtered(msg, points, valid, inlier_mask)

    def ransac_ego_velocity(self, directions, doppler):
        
        n = len(directions)
        if n < 3:
            return None, None

        best_inliers = None
        best_count = 0
        rng = np.random.default_rng()

        for _ in range(self.ransac_iterations):
            sample_idx = rng.choice(n, size=3, replace=False)
            H_sample = directions[sample_idx]
            d_sample = doppler[sample_idx]

            # H . v = -doppler  =>  v = lstsq(H, -doppler)
            try:
                v_candidate = np.linalg.lstsq(H_sample, -d_sample, rcond=None)[0]
            except np.linalg.LinAlgError:
                continue

            predicted = -directions @ v_candidate
            residuals = np.abs(doppler - predicted)
            inliers = residuals < self.inlier_threshold
            count = int(np.sum(inliers))

            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count < self.min_points:
            return None, None

        H_in = directions[best_inliers]
        d_in = doppler[best_inliers]
        velocity = np.linalg.lstsq(H_in, -d_in, rcond=None)[0]
        velocity = np.array([
            velocity[0],
            velocity[1],
            0.0,
        ])

        return velocity, best_inliers

    def publish_twist(self, velocity, header):
        msg = TwistWithCovarianceStamped()
        msg.header = header
        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])
        
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = 0.05  
        msg.twist.covariance = cov
        self.twist_pub.publish(msg)

    def integrate_and_publish(self, velocity, header):
        now = header.stamp.sec + header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0 or dt > 1.0:
            return  

        if self.last_imu_quat is not None:
            world_velocity = self.rotate_by_quat(velocity, self.last_imu_quat)
        else:
            world_velocity = velocity

        self.position += world_velocity * dt

        odom = Odometry()
        odom.header = header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "radar"
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])
        if self.last_imu_quat is not None:
            odom.pose.pose.orientation.x = self.last_imu_quat[0]
            odom.pose.pose.orientation.y = self.last_imu_quat[1]
            odom.pose.pose.orientation.z = self.last_imu_quat[2]
            odom.pose.pose.orientation.w = self.last_imu_quat[3]
        else:
            odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(velocity[0])
        odom.twist.twist.linear.y = float(velocity[1])
        odom.twist.twist.linear.z = float(velocity[2])
        self.odom_pub.publish(odom)

    @staticmethod
    def rotate_by_quat(v, q):
        x, y, z, w = q
        qv = np.array([x, y, z])
        t = 2.0 * np.cross(qv, v)
        return v + w * t + np.cross(qv, t)

    def publish_filtered(self, original_msg, points, valid_mask, inlier_mask):
        valid_points = points[valid_mask]
        kept = valid_points[inlier_mask]

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name=self.doppler_field, offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.stamp = original_msg.header.stamp
        header.frame_id = original_msg.header.frame_id

        kept_tuples = [
            (float(p["x"]), float(p["y"]), float(p["z"]), float(p[self.doppler_field]))
            for p in kept
        ]
        filtered_cloud = pc2.create_cloud(header, fields, kept_tuples)
        self.filtered_pub.publish(filtered_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RadarEgoVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()