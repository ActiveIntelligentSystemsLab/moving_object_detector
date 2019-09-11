# scene_flow_clusterer

A ROS package for clustering scene flow to detect moving objects.

Input scene flow is obtained from [scene_flow_constructor](https://github.com/ActiveIntelligentSystemsLab/moving_object_detector/tree/master/scene_flow_constructor).

## Node: scene_flow_clusterer_node

Clustering scene flow and publish each clusters.
Clustering is just based on pixel neighborhood relation.

### Subscribed topic

* `scene_flow` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

  Input scene flow(velocity vector in 3D-space).

  Type of each point is [PointXYZVelocity](https://github.com/ActiveIntelligentSystemsLab/moving_object_detector/blob/master/scene_flow_constructor/include/scene_flow_constructor/pcl_point_xyz_velocity.h).

  Also this PointCloud should be [organized](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#aca13e044f7064cd2114d37a42bdedc87).
  `organized` means that index of the points are aligned by width and height and corresponded to left image pixels. 

### Published topics

* `~clusters` ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))

  Store clusterd pointclouds as [Points Marker](http://wiki.ros.org/rviz/DisplayTypes/Marker#Points_.28POINTS.3D8.29).
  It can be visualized by [RViz](http://wiki.ros.org/rviz) but velocity element is not contained.

* `~clusters_image` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  Visualize clusters on image plane.
  
  Pixels of each cluster is filled by different color.

* `~moving_objects` ([moving_object_msgs/MovingObjectArray](https://github.com/ActiveIntelligentSystemsLab/moving_object_detector/blob/master/moving_object_msgs/msg/MovingObjectArray.msg))

  Mean velocity, center position and bounding box of each cluster.

### Parameters

Parmeters are define in [here](cfg/Clusterer.cfg).

It can be set by [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)

## Nodelet: scene_flow_clusterer/scene_flow_clusterer

Nodelet version of scene_flow_clusterer_node.

Topics and parameters are same to the node.
