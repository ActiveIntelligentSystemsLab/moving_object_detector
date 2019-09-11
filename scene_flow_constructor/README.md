# scene_flow_constructor

A ROS package to construct scene flow from camera transform, optical flow and disparity image.

## Node: scene_flow_constructor_node

Construct scene flow from camera transform, optical flow and disparity image.

This node should be used with input_synchronizer_node.

### Subscribed topics

* `camera_transform` ([geometry_msgs/TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html))

  Camera transform of left camera from `now` time to `previous` time, caused by the camera's ego motion.

  It can be obtained from [visual odometry node](https://github.com/fujimo-t/viso2).

* `optical_flow` ([optical_flow_msgs/DenseOpticalFlow](https://github.com/ActiveIntelligentSystemsLab/ros_optical_flow/blob/master/optical_flow_msgs/msg/DenseOpticalFlow.msg))

  Dense optical flow of left image between `now` frame and `previous` frame.

  It can be obtained from [optical flow node](https://github.com/ActiveIntelligentSystemsLab/pwc_net_ros).

* `disparity_image` ([stereo_msgs/DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html))

  Disparity image estimated from left and right images.

  It can be obtained from [stereo matching node](https://github.com/ActiveIntelligentSystemsLab/sgm_gpu_ros).

* `camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  Camera info of left camera.

### Published topic

* `~scene_flow` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

  Pointcloud with velocity vector in 3D-space.

  It's reference frame is left camera frame when `previous` time.
  
  Type of each point is [PointXYZVelocity](https://github.com/ActiveIntelligentSystemsLab/moving_object_detector/blob/master/scene_flow_constructor/include/scene_flow_constructor/pcl_point_xyz_velocity.h).

* `~synthetic_optical_flow` ([optical_flow_msgs/DenseOpticalFlow](https://github.com/ActiveIntelligentSystemsLab/ros_optical_flow/blob/master/optical_flow_msgs/msg/DenseOpticalFlow.msg))

  Output for debug.

  Synthetic optical flow calculated by camera transform, depth image and static scene assumption.

  Used inside of this node to distinguish dynamic pixels by comparing to `optical_flow`.

* `~optical_flow_residual` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Output for debug.

  Residual between `optical_flow` and `~synthetic_optical_flow`.

  Encoding is `mono8` (8bit, single channel, unsigned char).

