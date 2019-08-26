# Moving objects detection from stereo images

This contains ROS packages to detect moving objects from stereo images.

## Prerequisite

* ROS melodic
* Gazebo 9
* CUDA 10.1
* CuDNN 7

## Dependent packages

You should place following ros packages to your catkin workspaces and build.

* [ros_optical_flow](https://github.com/ActiveIntelligentSystemsLab/ros_optical_flow)
* [pwc_net](https://github.com/fujimo-t/pwc_net_ros)
* [sgm_gpu](https://github.com/ActiveIntelligentSystemsLab/sgm_gpu_ros)
* [disparity_visualize](https://github.com/ActiveIntelligentSystemsLab/disparity_visualize)
* [turtlebot3_stereo_description](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/fujimoto/turtlebot3_stereo_description)
* [mars_rover_description](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/fujimoto/mars_rover_description)
* [gazebo_factory_world](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/fujimoto/gazebo_factory_world)
* [viso2_ros](https://aisl-serv6.aisl.cs.tut.ac.jp:20443/fujimoto/viso2_ros_pub_outlier)

## Build

```shell
$ cd <YourCatkinWorkspace>/src
$ git clone https://aisl-serv6.aisl.cs.tut.ac.jp:20443/fujimoto/moving_object_detector.git
$ cd ../
$ catkin_make
```

## Testing

Launch Gazebo simulator first:

```shell
$ roslaunch gazebo_factory_world factory_world.launch
```

If this is first time of launch Gazebo, take a minite by downloading models.

After the simulator world is available, then spawn robots:

```shell
$ roslaunch moving_object_detector_launch gazebo_simulation_spawn_robots.launch
```

Then robots spawn in simulator world and it can controlled by Xbox wired controller.

Next, launch moving object detector:

```shell
$ roslaunch moving_object_detector_launch gazebo_simulation_process_nodelet.launch
```

You can see result by rqt:

```shell
$ rqt --perspective-file `rospack find moving_object_detector_launch`/rqt/moving_object_detector.perspective
```

## contained packages

* disparity_image_proc

  Small library to process stereo_msgs/DisparityImage

* moving_object_detector_launch

  Launch files

* moving_object_msgs

  Message definition represents moving objects

* moving_object_to_marker

  Convert moving_object_msgs to visualization_msgs/MarkerArray to visualize by RViz

* moving_object_tracker

  Track moving objects by Kalman filter

* project_moving_objects_on_image

  Visualize moving_object_msgs with input image

* velocity_estimator

  Generate pointcloud with velocity from camera transform, disparity image and optical flow

* velocity_pc_clusterer

  Clustering pointcloud with velocity

* kkl

  Kalman filter library written by Kenji Koide.
  It is copied from https://aisl-serv6.aisl.cs.tut.ac.jp:20443/koide/grace_person_following.git