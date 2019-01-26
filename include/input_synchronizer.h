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

class InputSynchronizer {
private:
  ros::NodeHandle node_handle_;
  
  ros::ServiceServer publish_service_;
  
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  
  image_transport::CameraPublisher left_rect_image_pub_;
  image_transport::CameraPublisher right_rect_image_pub_;
  
  image_transport::SubscriberFilter left_rect_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_rect_info_sub_;
  image_transport::SubscriberFilter right_rect_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_rect_info_sub_;
  
  typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> DataTimeSynchronizer;
  std::shared_ptr<DataTimeSynchronizer> time_sync_;
  
  sensor_msgs::CameraInfo left_camera_info_;
  sensor_msgs::Image left_rect_image_;
  sensor_msgs::CameraInfo left_rect_info_;
  sensor_msgs::Image right_rect_image_;
  sensor_msgs::CameraInfo right_rect_info_;
    
  void publish();
  
  void dataCallBack(const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info);
  bool publishServiceCallback(moving_object_detector::InputSynchronizerPublish::Request &request, moving_object_detector::InputSynchronizerPublish::Response &response);
  
public:
  InputSynchronizer();
};

#endif
