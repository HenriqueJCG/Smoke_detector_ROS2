# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    #bag_path = '/media/henrique/fruc-t9-1/agilex/2026_06_25_21_28_53__smokey1_' 
    #bag_path = '/media/henrique/fruc-t9-1/agilex/2026_06_25_21_44_40__nosmokey_'
    
    
    #bag_path ='/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/first-no-smoke'
    #bag_path ='/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/second-smoke'
    bag_path ='/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/third-continuos-smoke'

    current_pkg = FindPackageShare("genz_icp")

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_path',
            default_value=bag_path,
            description='Path to the ROS2 bag file'
        ),

        launch_ros.actions.Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver_node',
            output='screen',
        ),

        # --- GenZ-ICP Odometry (LiDAR) ---
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'genz_icp', 'odometry.launch.py',
                'topic:=/hesai/points'
                #'topic:=/aeva/ATLAS/point_cloud_compensated'
            ],
            output='screen',
        ),

        # --- KISS-ICP Odometry (Radar) ---
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'kiss_icp', 'odometry.launch.py',
                'topic:=/sensrad/radar_1/radar_data'
            ],
            output='screen',
        ),

        Node(
            package="point_ratio",
            executable="point_ratio",
            name="point_ratio",
            output="screen",
            parameters=[{"mode": "hesai"}],
        ),
        Node(
            package="point_ratio",
            executable="radar_text_marker",
            name="radar_text_marker",
            output="screen",
        ),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
        ),

        Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "genz_icp_ros2.rviz"])],
        ),

        # --- ROS2 Bag Play ---
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'),'--clock','--rate', '1.0'],
            output='screen',
        ),
])