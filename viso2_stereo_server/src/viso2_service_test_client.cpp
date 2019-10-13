#include "viso2_service_test_client.h"

#include <image_transport/camera_common.h>
#include <ros/ros.h>
#include <viso2_stereo_server/EstimateMotionFromStereo.h>

namespace viso2_stereo_server 
{
  Viso2ServiceTestClient::Viso2ServiceTestClient()
  {
    ros::NodeHandle node_handle;

    estimate_motion_client_ = node_handle.serviceClient<EstimateMotionFromStereo>("estimate_motion_from_stereo");
    estimate_motion_client_.waitForExistence();

    image_transport_.reset(new image_transport::ImageTransport(node_handle));

    // Subscribe image topics
    std::string left_image_topic = node_handle.resolveName("left_image");
    std::string right_image_topic = node_handle.resolveName("right_image");
    left_image_subscriber_.subscribe(*image_transport_, left_image_topic, 1);
    right_image_subscriber_.subscribe(*image_transport_, right_image_topic, 1);

    // Subscribe camera info topics
    std::string left_info_topic = image_transport::getCameraInfoTopic(left_image_topic);
    std::string right_info_topic = image_transport::getCameraInfoTopic(right_image_topic);
    left_info_subscriber_.subscribe(node_handle, left_info_topic, 1);
    right_info_subscriber_.subscribe(node_handle, right_info_topic, 1);

    // To synchronize timestamp of left/right image and camera info
    stereo_synchronizer_.reset(new StereoSynchronizer(left_image_subscriber_, right_image_subscriber_, left_info_subscriber_, right_info_subscriber_, 10));
    stereo_synchronizer_->registerCallback(&Viso2ServiceTestClient::stereoSynchronizerCallback, this);
  }

  void Viso2ServiceTestClient::stereoSynchronizerCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info)
  {
    // Set request
    EstimateMotionFromStereo estimate_motion_service;
    estimate_motion_service.request.left_image = *left_image;
    estimate_motion_service.request.right_image = *right_image;
    estimate_motion_service.request.left_camera_info = *left_camera_info;
    estimate_motion_service.request.right_camera_info = *right_camera_info;

    if (estimate_motion_client_.call(estimate_motion_service))
    {
      EstimateMotionFromStereo::Response& response = estimate_motion_service.response;
      if (response.first_call)
      {
        ROS_INFO_STREAM("This is first call so camera motion is not calculated\n" << 
                        "Current timestamp: " << response.timestamp_current << "\n" <<
                        "Runtime: " << response.runtime);
      }
      else
      {
        if (response.success)
        {
          geometry_msgs::Transform& camera_motion = response.left_previous_to_current;
          ROS_INFO_STREAM("Succeeded to estimate motion\n" << 
                          "Current timestamp: " << response.timestamp_current << "\n" <<
                          "Previous timestamp: " << response.timestamp_previous << "\n" <<
                          "Runtime: " << response.runtime << "\n" <<
                          "Translation x: " << camera_motion.translation.x << "\n" << 
                          "Translation y: " << camera_motion.translation.y << "\n" << 
                          "Translation z: " << camera_motion.translation.z << "\n" << 
                          "rotation x: " << camera_motion.rotation.x << "\n" << 
                          "rotation y: " << camera_motion.rotation.y << "\n" << 
                          "rotation z: " << camera_motion.rotation.z << "\n" << 
                          "rotation w: " << camera_motion.rotation.w);
        }
        else
        {
          ROS_ERROR_STREAM("Service is called but failed to calculate motion!\n" << 
                           "Current timestamp: " << response.timestamp_current << "\n" <<
                           "Previous timestamp: " << response.timestamp_previous << "\n" <<
                           "Runtime: " << response.runtime);
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call service!");
    }
  }
};
