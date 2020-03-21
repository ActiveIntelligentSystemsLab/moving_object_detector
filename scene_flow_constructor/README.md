# scene_flow_constructor

A ROS package to construct scene flow from stereo image sequeunce.

## Node: scene_flow_constructor

Construct scene flow from stereo image.

### Subscribed topics

Input stereo image and camera info.

* `left_image` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Should be remapped.

* `right_image` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Should be remapped.

* `<base topic of left_image>/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  Remapping isn't necessary.

  Automatically find topic name based on `left_image` topic.

* `<base topic of right_image>/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  Remapping isn't necessary.

  Automatically find topic name based on `right_image` topic.

### Published topics

* `~optical_flow` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Optical flow of left image estimated by PWC-Net.

  It's encoding is 32FC2 
  (32 bit float, two channels. first channel is x-axis, second is y-axis element of optical flow).

* `~depth` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Depth image estimated by SGM.

* `~scene_flow` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

  Pointcloud with velocity vector in 3D-space.
  
  Type of each point is [PointXYZVelocity](https://github.com/ActiveIntelligentSystemsLab/moving_object_detector/blob/master/scene_flow_constructor/include/scene_flow_constructor/pcl_point_xyz_velocity.h).

* `~synthetic_optical_flow` ([optical_flow_msgs/DenseOpticalFlow](https://github.com/ActiveIntelligentSystemsLab/ros_optical_flow/blob/master/optical_flow_msgs/msg/DenseOpticalFlow.msg))

  Output for debug.

  Synthetic optical flow calculated from camera transform, depth image and static scene assumption.

  Used inside of this node to distinguish dynamic pixels by comparing to `optical_flow`.

### Parameters

* `~image_transport` (string)

  See [here](http://wiki.ros.org/image_transport#Parameters-1).

Parameters are defined in [here](cfg/SceneFlowConstructor.cfg).

#### Dynamic parameters

List of parameters is [this](cfg/SceneFlowConstructor.cfg).

They can be set by [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure).

