# Moving objects detection from stereo images

This contains ROS packages to detect moving objects from stereo images.

## Prerequisite

### Hardware

* NVIDIA GPU
* [Xbox 360 Controller](https://www.microsoft.com/accessories/en-ww/products/gaming/xbox-360-controller-for-windows/52a-00004)

  To move a camera and a moving object in Gazebo simulator.
  
  This can be replaced with other controller supported by [joy package](http://wiki.ros.org/joy).
  But maybe change of key assignment is needed.

### Software

* [Docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Docker Compose](https://docs.docker.com/compose/install/)
* [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker#quickstart)
* [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker#quickstart)
  
  nvidia-docker2 is deprecated now but it is needed to use Docker Compose with NVIDIA GPU.

## Build

```shell
$ git clone https://github.com/ActiveIntelligentSystemsLab/moving_object_detector.git
$ cd moving_object_detector/docker
$ sudo docker-compose build
$ sudo docker-compose up --no-start
```

## Run

1. Launch commands at `moving_object_detector/docker` directory:

   ```shell
   $ xhost +local:root
   $ sudo docker-compose start
   ```

   Then four containers are launched:
   * ROS master
   * RViz
   * rqt
   * Terminal
     * To run nodes and commands, edit files
     * Installed tools: byobu(tmux), gdb, htop and vim

2. Stop containers by:

   ```shell
   $ sudo docker-compose stop
   ```

### Detection in Gazebo simulator

Plug Xbox controller and run below command in the terminal:

```shell
$ roslaunch moving_object_detector_launch gazebo_sim.launch
```

And open new terminal tab (or use byobu) and run below command:

```shell
$ roslaunch moving_object_detector_launch gazebo_moving_object_detection.launch
```

Move stereo camera by A+Left stick and move an object by A+Right stick.

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

* scene_flow_constructor

  Construct scene flow from camera transform, disparity image and optical flow

* scene_flow_clusterer

  Clustering scene flow and each clusters are treated as moving objects

* kkl

  Kalman filter library written by Kenji Koide.

* viso2_stereo_server

  Provide service which estimate camera motion from stereo image

