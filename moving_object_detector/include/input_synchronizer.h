#ifndef __HEADER_INPUT_SYNCHRONIZER__
#define __HEADER_INPUT_SYNCHRONIZER__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <moving_object_detector/InputSynchronizerPublish.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <viso2_ros/VisoInfo.h>
#include <optical_flow_msg/OpticalFlow.h>
#include <stereo_msgs/DisparityImage.h>

#include <memory>

// visual odometry, stereo matching, optical flowへの入力を与えるノード
// visual odometry, stereo matching, optical flowの処理が終了するたびに，タイムスタンプを同期させたステレオ画像とcamera infoを1セットpublishする
class InputSynchronizer {
private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<ros::NodeHandle> private_node_handle;
  
  ros::ServiceServer publish_service_;
  
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  
  // Publisher for synchronized stereo images and camera infos
  image_transport::CameraPublisher left_rect_image_pub_;
  image_transport::CameraPublisher right_rect_image_pub_;
  
  // Subscribers for unsynchronized stereo images and camera infos
  image_transport::SubscriberFilter left_rect_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_rect_info_sub_;
  image_transport::SubscriberFilter right_rect_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_rect_info_sub_;
  
  // TImeSynchronizer for stereo images and camera info
  typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> StereoSynchronizer;
  std::shared_ptr<StereoSynchronizer> stereo_time_sync_;

  // Subscriber to check stereo matching, optical flow and visual odometry is completed
  message_filters::Subscriber<viso2_ros::VisoInfo> viso2_info_sub_;
  message_filters::Subscriber<optical_flow_msg::OpticalFlow> optical_flow_left_sub_;
  message_filters::Subscriber<optical_flow_msg::OpticalFlow> optical_flow_right_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;

  // TimeSynchronizer to check stereo matching, optical flow and visual odometry is completed
  typedef message_filters::TimeSynchronizer<viso2_ros::VisoInfo, optical_flow_msg::OpticalFlow, optical_flow_msg::OpticalFlow, stereo_msgs::DisparityImage> ProcessedDataSynchronizer;
  std::shared_ptr<ProcessedDataSynchronizer> processed_data_time_sync_;
  
  sensor_msgs::CameraInfo left_camera_info_;
  sensor_msgs::Image left_rect_image_;
  sensor_msgs::CameraInfo left_rect_info_;
  sensor_msgs::Image right_rect_image_;
  sensor_msgs::CameraInfo right_rect_info_;

  double republish_timeout_;
  ros::Time last_published_stamp_;
  bool publish_required_;
    
  void publishSynchronizedStereo();
  
  void stereoTimeSyncCallback(const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info);
  void processedDataSyncCallback(const viso2_ros::VisoInfoConstPtr& viso2_info, const optical_flow_msg::OpticalFlowConstPtr& left_flow, const optical_flow_msg::OpticalFlowConstPtr& right_flow, const stereo_msgs::DisparityImageConstPtr& disparity);
  bool publishServiceCallback(moving_object_detector::InputSynchronizerPublish::Request &request, moving_object_detector::InputSynchronizerPublish::Response &response);
  
public:
  InputSynchronizer();
};

#endif
