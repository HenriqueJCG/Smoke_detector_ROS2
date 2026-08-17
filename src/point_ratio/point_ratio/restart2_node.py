#!/usr/bin/env python3

import os
import signal
import subprocess
import time
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointCloud


class KissManager(Node):

    def __init__(self):
        super().__init__("kiss_manager")

        self.running = True
        self.proc = None
        self.proc2 = None
        self.last_pose_msg = None
        self.path = Path()

        self.declare_parameter("mode", "hesai")
        self.declare_parameter("stop_lidar_threshold", 0.2)
        self.declare_parameter("start_lidar_threshold", 0.7)
        self.mode = self.get_parameter("mode").value

        self.topic2 = "/filtered_hugin"

        if self.mode == "hesai":
            self.topic = "/filtered_hesai"
        else:
            self.topic = "/filtered_aeva"

        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.create_subscription(Float64,"/point_ratio",self.callback,10)
        self.create_subscription(Odometry,"/odometry/filtered",self.pose_callback,10)
        self.create_subscription(Odometry,"/kiss/odometry",self.radar_pose_callback,10)
        self.create_subscription(Odometry,"/genz/odometry",self.lidar_pose_callback,10)


    def pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

    def radar_pose_callback(self, msg):
        self.last_pose_msg = msg


    def lidar_pose_callback(self, msg):
        self.last_pose_msg = msg

    def start_kiss(self):

        if (self.running and self.proc is not None and self.proc.poll() is None):
            return

        if self.last_pose_msg is None:
            pose_args = [
                "use_initial_pose:=true",
                "initial_pose_x:=0.0",
                "initial_pose_y:=0.0",
                "initial_pose_z:=0.0",
                "initial_pose_qx:=0.0",
                "initial_pose_qy:=0.0",
                "initial_pose_qz:=0.0",
                "initial_pose_qw:=1.0",
            ]
        else:
            p = self.last_pose_msg.pose.pose
            pose_args = [
                "use_initial_pose:=true",
                f"initial_pose_x:={p.position.x}",
                f"initial_pose_y:={p.position.y}",
                f"initial_pose_z:={p.position.z}",
                f"initial_pose_qx:={p.orientation.x}",
                f"initial_pose_qy:={p.orientation.y}",
                f"initial_pose_qz:={p.orientation.z}",
                f"initial_pose_qw:={p.orientation.w}",
            ]
 
        self.proc = subprocess.Popen(
            [
                "ros2", "launch", "genz_icp", "odometry.launch.py",
                f"topic:={self.topic}",
            ] + pose_args,  
            start_new_session=True,
        )

        self.running = True


    def stop_kiss(self):
        if not self.running or self.proc is None:
            return
 
        pgid = os.getpgid(self.proc.pid)
 
        os.killpg(pgid, signal.SIGINT)
        try:
            self.proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("genz_icp did not exit after SIGINT, killing process group")
            os.killpg(pgid, signal.SIGKILL)
            self.proc.wait()
 
        if self._group_still_alive(pgid):
            self.get_logger().warn("Child nodes still alive after parent exit, forcing group shutdown")
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
 
        self.proc = None
        self.running = False

    def start_genz(self):

        if (not self.running and self.proc2 is not None and self.proc2.poll() is None):
            return

        if self.last_pose_msg is None:
            pose_args = [
                "use_initial_pose:=true",
                "initial_pose_x:=0.0",
                "initial_pose_y:=0.0",
                "initial_pose_z:=0.0",
                "initial_pose_qx:=0.0",
                "initial_pose_qy:=0.0",
                "initial_pose_qz:=0.0",
                "initial_pose_qw:=1.0",
            ]
        else:
            p = self.last_pose_msg.pose.pose
            pose_args = [
                "use_initial_pose:=true",
                f"initial_pose_x:={p.position.x}",
                f"initial_pose_y:={p.position.y}",
                f"initial_pose_z:={p.position.z}",
                f"initial_pose_qx:={p.orientation.x}",
                f"initial_pose_qy:={p.orientation.y}",
                f"initial_pose_qz:={p.orientation.z}",
                f"initial_pose_qw:={p.orientation.w}",
            ]
 
        self.proc2 = subprocess.Popen(
            [
                "ros2", "launch", "kiss_icp", "odometry.launch.py",
                f"topic:={self.topic2}",
            ] + pose_args,  
            start_new_session=True,
        )

        self.running = False


    def stop_genz(self):
        if not self.running or self.proc2 is None:
            return
 
        pgid = os.getpgid(self.proc2.pid)
 
        os.killpg(pgid, signal.SIGINT)
        try:
            self.proc2.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.get_logger().warn("genz_icp did not exit after SIGINT, killing process group")
            os.killpg(pgid, signal.SIGKILL)
            self.proc2.wait()
 
        if self._group_still_alive(pgid):
            self.get_logger().warn("Child nodes still alive after parent exit, forcing group shutdown")
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
 
        self.proc2 = None
        self.running = True
 
    @staticmethod
    def _group_still_alive(pgid, retries=10, delay=0.5):
        for _ in range(retries):
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return False
            time.sleep(delay)
        return True

    def callback(self, msg):

        if msg.data <= self.get_parameter("stop_lidar_threshold").value:
            self.stop_kiss()
            self.start_genz()

        elif msg.data >= self.get_parameter("start_lidar_threshold").value:
            self.stop_genz()
            self.start_kiss()

    def destroy_node(self):
        self.stop_kiss()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = KissManager()

    node.start_kiss()
    node.stop_genz()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()