#!/usr/bin/env python3

import os
import signal
import subprocess
import time
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped


class KissManager(Node):

    def __init__(self):
        super().__init__("kiss_manager")

        self.running = False
        self.proc = None
        self.last_pose_msg = None

        self.declare_parameter("mode", "hesai")
        self.mode = self.get_parameter("mode").value

        if self.mode == "hesai":
            self.topic = "/hesai/points"
        else:
            self.topic = "/aeva/ATLAS/point_cloud_compensated"

        self.create_subscription(Float64,"/point_ratio",self.callback,10)
        self.create_subscription(Odometry,"/odometry/filtered",self.pose_callback,10)

        self.set_pose_client = self.create_client(SetPose,"/set_pose")


    def pose_callback(self, msg):
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
                f"initial_pose_x:={p.position.y}",
                f"initial_pose_y:={-p.position.x}",
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
            preexec_fn=os.setsid,  
        )

        if self.last_pose_msg is not None:
            self.call_set_pose(self.last_pose_msg)

        self.running = True

    def call_set_pose(self, pose_msg):
        if not self.set_pose_client.service_is_ready(): 
            return
 
        request = SetPose.Request()
        msg = PoseWithCovarianceStamped()
        msg.header = pose_msg.header
        msg.pose = pose_msg.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = cov[21] = cov[28] = cov[35] = 0.05
        msg.pose.covariance = cov
        request.pose = msg
 
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(self._on_set_pose_response)
 
    def _on_set_pose_response(self, future):
        try:
            future.result()
            self.get_logger().info("/set_pose call completed")
        except Exception as e:
            self.get_logger().warn(f"/set_pose call failed: {e}")

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

        if msg.data <= 0.4:
            self.stop_kiss()

        elif msg.data >= 0.5:
            self.start_kiss()

    def destroy_node(self):
        self.stop_kiss()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = KissManager()

    node.start_kiss()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()