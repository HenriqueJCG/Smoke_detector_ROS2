# ROS2 Radar Perception Code

Repository for the code related to the Radar based EKF LiDAR-Radar fusion developed for ROS2 Jazzy.

---

### [KISS-ICP](https://github.com/PRBonn/kiss-icp)

**AgileX**

```ros2 launch kiss_icp odometry.launch.py topic:=/sensrad/radar_1/radar_data```

```ros2 launch kiss_icp odometry.launch.py topic:=/aeva/ATLAS/point_cloud_compensated```



```ros2 run kiss_icp kiss_icp_node --ros-args --params-file ~/ros2_ws/src/kiss-icp/ros/config/kiss_config_hesai.yaml -r pointcloud_topic:=/hesai/points```

```ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id hesai_lidar```


### [GENZ-ICP](https://github.com/cocel-postech/genz-icp)

**AgileX**

```ros2 launch genz_icp odometry.launch.py topic:=/hesai/points```


### [EKF](https://github.com/cra-ros-pkg/robot_localization/tree/noetic-devel)

```ros2 launch robot_localization ekf.launch.py```

---

**Youtube Playlist:** [https://youtube.com/playlist?list=PL5dDSTSIq7yJWkVsNU3b_be_bDQzBtLgO&si=xRb-qyhVP9_qFgAG](https://youtube.com/playlist?list=PL5dDSTSIq7yJWkVsNU3b_be_bDQzBtLgO&si=xRb-qyhVP9_qFgAG)