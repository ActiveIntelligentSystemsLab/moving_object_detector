#include "input_synchronizer.h"

InputSynchronizer::InputSynchronizer()
{
  image_transport_ = std::make_shared<image_transport::ImageTransport>(node_handle_);
  
  std::string publish_left_rect_image_topic = node_handle_.resolveName("synchronizer_output_left_rect_image");
  std::string publish_right_rect_image_topic = node_handle_.resolveName("synchronizer_output_right_rect_image");
  
  left_rect_image_pub_ = image_transport_->advertiseCamera(publish_left_rect_image_topic, 1);
  right_rect_image_pub_ = image_transport_->advertiseCamera(publish_right_rect_image_topic, 1);
  
  std::string subscribe_left_rect_image_topic = node_handle_.resolveName("synchronizer_input_left_rect_image");
  std::string subscribe_right_rect_image_topic = node_handle_.resolveName("synchronizer_input_right_rect_image");
  
  left_rect_image_sub_.subscribe(*image_transport_, subscribe_left_rect_image_topic, 10);
  left_rect_info_sub_.subscribe(node_handle_, image_transport::getCameraInfoTopic(subscribe_left_rect_image_topic), 10);
  right_rect_image_sub_.subscribe(*image_transport_, subscribe_right_rect_image_topic, 10);
  right_rect_info_sub_.subscribe(node_handle_, image_transport::getCameraInfoTopic(subscribe_right_rect_image_topic), 10);
  time_sync_ = std::make_shared<DataTimeSynchronizer>(left_rect_image_sub_, left_rect_info_sub_, right_rect_image_sub_, right_rect_info_sub_, 10);
  time_sync_->registerCallback(boost::bind(&InputSynchronizer::dataCallBack, this, _1, _2, _3, _4));
  
  publish_service_ = node_handle_.advertiseService("input_synchronizer_publish", &InputSynchronizer::publishServiceCallback, this);
}

void InputSynchronizer::dataCallBack(const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info)
{
  static int count = 0;
  
  left_rect_image_ = *left_rect_image;
  left_rect_info_ = *left_rect_info;
  right_rect_image_ = *right_rect_image;
  right_rect_info_ = *right_rect_info;
}

bool InputSynchronizer::publishServiceCallback(moving_object_detector::InputSynchronizerPublish::Request &request, moving_object_detector::InputSynchronizerPublish::Response &response)
{
  publish();
  return true;
}

void InputSynchronizer::publish()
{
  left_rect_image_pub_.publish(left_rect_image_, left_rect_info_);
  right_rect_image_pub_.publish(right_rect_image_, right_rect_info_);
}
