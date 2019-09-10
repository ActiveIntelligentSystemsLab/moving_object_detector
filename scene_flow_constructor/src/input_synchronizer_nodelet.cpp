#include <pluginlib/class_list_macros.h>

#include "input_synchronizer_nodelet.h"

PLUGINLIB_EXPORT_CLASS(scene_flow_constructor::InputSynchronizerNodelet, nodelet::Nodelet)

namespace scene_flow_constructor {

void InputSynchronizerNodelet::onInit()
{
  ros::NodeHandle &node_handle = getNodeHandle();
  ros::NodeHandle &private_node_handle = getPrivateNodeHandle();
  image_transport_.reset(new image_transport::ImageTransport(node_handle));

  republish_timeout_ = private_node_handle.param("republish_timeout", 1.0);
  
  // 同期済みステレオ画像とcamera infoのpublisherを初期化
  std::string publish_left_rect_image_topic = node_handle.resolveName("synchronizer_output_left_rect_image");
  std::string publish_right_rect_image_topic = node_handle.resolveName("synchronizer_output_right_rect_image");
  left_rect_image_pub_ = image_transport_->advertiseCamera(publish_left_rect_image_topic, 1);
  right_rect_image_pub_ = image_transport_->advertiseCamera(publish_right_rect_image_topic, 1);
  
  // 同期前のステレオ画像とcamera infoの，subscriberとTimeSynchronizerを初期化
  std::string subscribe_left_rect_image_topic = node_handle.resolveName("synchronizer_input_left_rect_image");
  std::string subscribe_right_rect_image_topic = node_handle.resolveName("synchronizer_input_right_rect_image");
  left_rect_image_sub_.subscribe(*image_transport_, subscribe_left_rect_image_topic, 10);
  left_rect_info_sub_.subscribe(node_handle, image_transport::getCameraInfoTopic(subscribe_left_rect_image_topic), 10);
  right_rect_image_sub_.subscribe(*image_transport_, subscribe_right_rect_image_topic, 10);
  right_rect_info_sub_.subscribe(node_handle, image_transport::getCameraInfoTopic(subscribe_right_rect_image_topic), 10);
  stereo_time_sync_ = std::make_shared<StereoSynchronizer>(left_rect_image_sub_, left_rect_info_sub_, right_rect_image_sub_, right_rect_info_sub_, 10);
  stereo_time_sync_->registerCallback(boost::bind(&InputSynchronizerNodelet::stereoTimeSyncCallback, this, _1, _2, _3, _4));

  // 各処理結果を受け取るsubscriberの初期化
  viso2_info_sub_.subscribe(node_handle, "viso2_info", 1);
  optical_flow_left_sub_.subscribe(node_handle, "optical_flow_left", 1);
  disparity_image_sub_.subscribe(node_handle, "disparity_image", 1);
  processed_data_time_sync_ = std::make_shared<ProcessedDataSynchronizer>(viso2_info_sub_, optical_flow_left_sub_, disparity_image_sub_, 30);
  processed_data_time_sync_->registerCallback(boost::bind(&InputSynchronizerNodelet::processedDataSyncCallback, this, _1, _2, _3));
  
  publish_service_ = node_handle.advertiseService("input_synchronizer_publish", &InputSynchronizerNodelet::publishServiceCallback, this);
}

void InputSynchronizerNodelet::stereoTimeSyncCallback(const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info)
{
  left_rect_image_ = *left_rect_image;
  left_rect_info_ = *left_rect_info;
  right_rect_image_ = *right_rect_image;
  right_rect_info_ = *right_rect_info;

  ros::Time current_stamp = left_rect_image->header.stamp;
  if (current_stamp - last_published_stamp_ >= ros::Duration(republish_timeout_))
  {
    NODELET_INFO("Republish for timeout: last publish at %f and current stamp is %f", last_published_stamp_.toSec(), current_stamp.toSec());
    publish_required_ = true;
  }

  if (publish_required_) {
    publishSynchronizedStereo();
    publish_required_ = false;
  }
}

void InputSynchronizerNodelet::processedDataSyncCallback(const viso2_ros::VisoInfoConstPtr& viso2_info, const optical_flow_msgs::DenseOpticalFlowConstPtr& left_flow, const stereo_msgs::DisparityImageConstPtr& disparity)
{
  publish_required_ = true;
}

bool InputSynchronizerNodelet::publishServiceCallback(scene_flow_constructor::InputSynchronizerPublish::Request &request, scene_flow_constructor::InputSynchronizerPublish::Response &response)
{
  NODELET_INFO("Repubish is required from service");
  publish_required_ = true;
  return true;
}

void InputSynchronizerNodelet::publishSynchronizedStereo()
{
  left_rect_image_pub_.publish(left_rect_image_, left_rect_info_);
  right_rect_image_pub_.publish(right_rect_image_, right_rect_info_);

  last_published_stamp_ = left_rect_image_.header.stamp;
}

}
