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

import launch_ros.actions
import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#Choose bag file to play
def generate_launch_description():

    def launch_setup(context, *args, **kwargs):

        dataset = LaunchConfiguration("dataset").perform(context)
        rate = LaunchConfiguration("rate").perform(context)

        bag_paths = {
            "polo1": "/media/henrique/T9/Datasets AgileX/PoloII_26-06-2026/nosmokey",
            "polo2": "/media/henrique/T9/Datasets AgileX/PoloII_26-06-2026/smokey",
            "pinhal1": "/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/first-no-smoke",
            "pinhal2": "/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/second-smoke",
            "pinhal3": "/media/henrique/T9/Datasets AgileX/PinhalMarrocos_04-06-2026/third-continuos-smoke",
        }

        if dataset not in bag_paths:
            raise RuntimeError(
                f"Unknown dataset '{dataset}'. "
                f"Choose one of: {', '.join(bag_paths.keys())}"
            )

        bag_path = bag_paths[dataset]

        return [
            ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path,'--clock','--rate', rate],
            output='screen',
            )
        ]

    current_pkg = FindPackageShare("genz_icp")

    return LaunchDescription([

        DeclareLaunchArgument("egovelocity",default_value="true"),
        DeclareLaunchArgument("point_algorithm",default_value="fov"),
        DeclareLaunchArgument("fusion",default_value="true"),
        DeclareLaunchArgument("lidar",default_value="false"),
        DeclareLaunchArgument("radar",default_value="true"),  
        DeclareLaunchArgument("LiDAR_used",default_value="hesai"),
        DeclareLaunchArgument("dataset",default_value="polo2"),
        DeclareLaunchArgument("rate",default_value="0.8"),
        DeclareLaunchArgument("start_lidar_threshold",default_value="0.7"),
        DeclareLaunchArgument("stop_lidar_threshold",default_value="0.2"),

        #Launch Hesai driver node for lidar pointcloud
        launch_ros.actions.Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver_node',
            output='screen',
        ),

        #GenZ-ICP Odometry node (LiDAR/FMCW LiDAR) 
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'kiss_icp', 'odometry.launch.py',
                'topic:=/hesai/points'
                #'topic:=/aeva/ATLAS/point_cloud_compensated'
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration("lidar")),
        ),

        #KISS-ICP Odometry node (Radar) 
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'kiss_icp', 'odometry.launch.py',
                'topic:=/sensrad/radar_1/radar_data'
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration("radar")),
        ),

        #Smoke probability node based on points from pointcloud
        launch_ros.actions.Node(
            package="point_ratio",
            executable="point_ratio",
            name="point_ratio",
            output="screen",
            parameters=[{"mode": "hesai"}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("point_algorithm"), "' == 'fov'"])),
        ),

        launch_ros.actions.Node(
            package="point_ratio",
            executable="point_ratio_kdtree",
            name="point_ratio",
            output="screen",
            parameters=[{"mode": "hesai"}],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("point_algorithm"), "' == 'kdtree'"])),
        ),
        


        #Ego velocity estimation with Radaar/ FMCW LiDARs
        launch_ros.actions.Node(
            package="point_ratio",
            executable="egovelocity",
            name="egovelocity",
            output="screen",
            condition=IfCondition(LaunchConfiguration("egovelocity")),
        ),

        #Text marker node that publishes messages to visualize the point ratio in RViz2
        launch_ros.actions.Node(
            package="point_ratio",
            executable="radar_text_marker",
            name="radar_text_marker",
            output="screen",
        ),

        #Node that computes the covariances of the odometry messages based on the point ratio
        launch_ros.actions.Node(
            package="point_ratio",
            executable="covariances",
            name="covariances",
            output="screen",
            condition=IfCondition(LaunchConfiguration("fusion")),
        ),

        #Node that restarts the lidar node based on the point ratio
        launch_ros.actions.Node(
            package="point_ratio",
            executable="restart_node",
            name="restart_node",
            parameters=[{"start_lidar_threshold": LaunchConfiguration("start_lidar_threshold"), "stop_lidar_threshold": LaunchConfiguration("stop_lidar_threshold")}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("fusion")),
        ),

        #EKF node for sensor fusion
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
            condition=IfCondition(LaunchConfiguration("fusion")),
        ),


        #Vizualisation of the results in RViz2
        launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                output={"both": "log"},
                arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "genz_icp_ros2.rviz"])],
        ),

        # Play selected bag file with selected rate
        OpaqueFunction(function=launch_setup)
])