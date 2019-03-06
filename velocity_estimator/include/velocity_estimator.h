#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <disparity_image_proc/disparity_image_processor.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <optical_flow_msg/OpticalFlow.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <velocity_estimator/VelocityEstimatorConfig.h>

#include <list>

class VelocityEstimator {
public:
  VelocityEstimator();
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher pc_with_velocity_pub_;
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub_;
  message_filters::Subscriber<optical_flow_msg::OpticalFlow> optical_flow_left_sub_;
  message_filters::Subscriber<optical_flow_msg::OpticalFlow> optical_flow_right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;

  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, optical_flow_msg::OpticalFlow, optical_flow_msg::OpticalFlow, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>> time_sync_;
  
  dynamic_reconfigure::Server<velocity_estimator::VelocityEstimatorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<velocity_estimator::VelocityEstimatorConfig>::CallbackType reconfigure_func_;
  
  double matching_tolerance_;

  stereo_msgs::DisparityImageConstPtr disparity_image_previous_;
  ros::Time time_stamp_previous_;
      
  void dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_left, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image);
  bool getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const cv::Mat &flow);
  bool getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor);
  bool isValid(const pcl::PointXYZ &point);
  template <typename PointT> void publishPointcloud(const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp);
  void reconfigureCB(velocity_estimator::VelocityEstimatorConfig& config, uint32_t level);
  void transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &now_to_previous);
};

#endif
