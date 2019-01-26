#ifndef __HEADER_MOVING_OBJECT_DETECTOR__
#define __HEADER_MOVING_OBJECT_DETECTOR__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <stereo_msgs/DisparityImage.h>
#include <dynamic_reconfigure/server.h>
#include <moving_object_detector/VelocityEstimatorConfig.h>
#include <list>

class VelocityEstimator {
public:
  VelocityEstimator();
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher pc_with_velocity_pub_;
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub_;
  message_filters::Subscriber<sensor_msgs::Image> optical_flow_left_sub_;
  message_filters::Subscriber<sensor_msgs::Image> optical_flow_right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;

  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>> time_sync_;
  
  dynamic_reconfigure::Server<moving_object_detector::VelocityEstimatorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_detector::VelocityEstimatorConfig>::CallbackType reconfigure_func_;

  ros::ServiceClient input_publish_client_;
  
  int downsample_scale_;
  double matching_tolerance_;

  stereo_msgs::DisparityImageConstPtr disparity_image_previous_;
  ros::Time time_stamp_previous_;
    
  bool first_run_;
  
  void reconfigureCB(moving_object_detector::VelocityEstimatorConfig& config, uint32_t level);
  void dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const sensor_msgs::ImageConstPtr& optical_flow_left, const sensor_msgs::ImageConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image);
};

#endif
