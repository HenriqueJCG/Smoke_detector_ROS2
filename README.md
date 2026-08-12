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


### [EKF](https://github.com/cra-ros-pkg/robot_localization/tree/jazzy-devel)

Basic launch: ```ros2 launch robot_localization ekf.launch.py```

Full launch example: ```ros2 launch robot_localization ekf.launch.py LiDAR_used:=aeva egovelocity:=true radar:=false fusion:=true point_algorithm:=fov lidar:=false dataset:=polo2 rate:=0.5```

**Customizable parameters:**

Seelct dataset: dataset (pinhal1/pinhal2/pinhal3/polo1/polo2)

Select LiDAR to be used: LiDAR_used (hesai/aeva)

Use EKF fusion: fusion (true/false)

Use RADAR ICP odometry: radar (true/false)

Use LiDAR ICP odometry (Fusion already starts this): lidar (false/true)

Use RADAR ego velocity estimation: egovelocity (false/true)

Select detection algorithm: point_algorithm (fov/kdtree)

Select bag replay speed: rate (0.1>)
---

**Youtube Playlist:** [https://youtube.com/playlist?list=PL5dDSTSIq7yJWkVsNU3b_be_bDQzBtLgO&si=xRb-qyhVP9_qFgAG](https://youtube.com/playlist?list=PL5dDSTSIq7yJWkVsNU3b_be_bDQzBtLgO&si=xRb-qyhVP9_qFgAG)