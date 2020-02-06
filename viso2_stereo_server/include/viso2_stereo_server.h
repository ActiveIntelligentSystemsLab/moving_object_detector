#ifndef VISO2_STEREO_SERVER_H_
#define VISO2_STEREO_SERVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <viso_stereo.h>
#include <viso2_stereo_server/EstimateMotionFromStereo.h>

namespace viso2_stereo_server
{
/**
 * \brief A class provides EstimateMotionFromStereo service 
 *
 * The service return motion of left camera between current frame and previous frame
 */
class Viso2StereoServer
{
private:
  /**
   * \brief Stereo visual odometry class from libviso2
   */
  std::shared_ptr<VisualOdometryStereo> visual_odometer_;
  /**
   * \brief Parameters used to initialize visual_odometer_
   */
  VisualOdometryStereo::parameters visual_odometer_params_;

  /**
   * \brief True if the service call is first
   *
   * First service call don't estimate camera motion because previous frame data is nothing
   */
  bool first_service_call_;

  /**
   * \brief Timestamp of previous stereo images
   */
  ros::Time previous_timestamp_;

  /**
   * \brief ROS service server which provide EstimateMotionFromStereo service
   */
  ros::ServiceServer motion_service_server_;

  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  std::string sensor_frame_id_;

  /**
   * \brief Callback function of motion_service_server_
   */
  bool motionServiceCallback(EstimateMotionFromStereo::Request& request, EstimateMotionFromStereo::Response& response);

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Publisher matchpoints_image_pub_;
  ros::Publisher pose_pub_;

  tf2::Transform integrated_pose_;

  /**
   * \brief Initialize visual_odometer_
   */
  void initOdometer(const sensor_msgs::CameraInfo& l_info_msg, const sensor_msgs::CameraInfo& r_info_msg);

  /**
   * \brief Publish TF between odom and base_link
   */
  void integrateAndBroadcastTF(const tf2::Transform& delta_transform, const ros::Time& timestamp);
  void publishMatchPointsImage(const sensor_msgs::Image& left_image);

public:
  Viso2StereoServer();

};

} // end of namespace

#endif

