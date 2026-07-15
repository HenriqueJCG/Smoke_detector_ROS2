#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rclpy.parameter_client import AsyncParametersClient


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

        self.param_client = AsyncParametersClient(self,"/genz_icp")

    def pose_callback(self, msg):
        self.last_pose_msg = msg

    def set_initial_pose_params(self, pose):

        params = []

        def add(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value)
            )
            params.append(p)

        add("initial_pose_x", pose["x"])
        add("initial_pose_y", pose["y"])
        add("initial_pose_z", pose["z"])

        add("initial_pose_qx", pose["qx"])
        add("initial_pose_qy", pose["qy"])
        add("initial_pose_qz", pose["qz"])
        add("initial_pose_qw", pose["qw"])

        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for parameter service...")

        future = self.param_client.set_parameters(params)
        rclpy.spin_until_future_complete(self, future)

    def start_kiss(self):

        if (self.running and self.proc is not None and self.proc.poll() is None):
            return

        if self.last_pose_msg is None:

            pose = {"x": 0.0,"y": 0.0,"z": 0.0,"qx": 0.0,"qy": 0.0,"qz": 0.0,"qw": 1.0,}

        else:

            p = self.last_pose_msg.pose.pose
            pose = {"x": p.position.x,"y": p.position.y,"z": p.position.z,"qx": p.orientation.x,"qy": p.orientation.y,"qz": p.orientation.z,"qw": p.orientation.w,}

        self.set_initial_pose_params(pose)

        self.proc = subprocess.Popen([
            "ros2",
            "launch",
            "genz_icp",
            "odometry.launch.py",
            f"topic:={self.topic}"
        ])

        if self.last_pose_msg is not None:

            while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for /set_pose service...")

            msg = PoseWithCovarianceStamped()
            msg.header = self.last_pose_msg.header
            msg.pose = self.last_pose_msg.pose

            msg.pose.covariance = [0.0] * 36
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[14] = 0.05
            msg.pose.covariance[21] = 0.05
            msg.pose.covariance[28] = 0.05
            msg.pose.covariance[35] = 0.05

            request = SetPose.Request()
            request.pose = msg

            future = self.set_pose_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

        self.running = True

    def stop_kiss(self):

        if not self.running:
            return

        if self.proc is not None:

            self.proc.kill()
            self.proc.wait()
            self.proc = None

        self.running = False

    def callback(self, msg):

        if msg.data <= 0.3:
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