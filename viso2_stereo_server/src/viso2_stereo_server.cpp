#include "viso2_stereo_server.h"
#include "odometry_params.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace viso2_stereo_server
{
  Viso2StereoServer::Viso2StereoServer()
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);
    local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
    local_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));

    motion_service_server_ = local_nh.advertiseService("estimate_motion_from_stereo", &Viso2StereoServer::motionServiceCallback, this);

    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
  }

  void Viso2StereoServer::initOdometer(const sensor_msgs::CameraInfo& l_info_msg, const sensor_msgs::CameraInfo& r_info_msg)
  {
    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(l_info_msg, r_info_msg);
    visual_odometer_params_.base      = model.baseline();
    visual_odometer_params_.calib.cu  = model.left().cx();
    visual_odometer_params_.calib.cv  = model.left().cy();
    visual_odometer_params_.calib.f   = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    ROS_INFO_STREAM("Initialized libviso2 stereo odometry with the following parameters:\n" << visual_odometer_params_);

    bool first_service_call_ = true;
  }

  bool Viso2StereoServer::motionServiceCallback(EstimateMotionFromStereo::Request& request, EstimateMotionFromStereo::Response& response)
  {
    ros::WallTime start_time = ros::WallTime::now();

    ROS_INFO("EstimateMotionFromStereo service is called");

    if (!visual_odometer_)
      initOdometer(request.left_camera_info, request.right_camera_info);

    // convert images
    cv_bridge::CvImageConstPtr cv_left = cv_bridge::toCvCopy(request.left_image, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr cv_right = cv_bridge::toCvCopy(request.right_image, sensor_msgs::image_encodings::MONO8);

    ROS_ASSERT(cv_left->image.step[0] == cv_right->image.step[0]);
    ROS_ASSERT(cv_left->image.rows == cv_right->image.rows);
    ROS_ASSERT(cv_left->image.cols == cv_left->image.cols);

    int32_t dims[] = {cv_left->image.cols, cv_left->image.rows, static_cast<int32_t>(cv_left->image.step[0])};
    response.success = visual_odometer_->process(cv_left->image.data, cv_right->image.data, dims);

    response.timestamp_current = request.left_image.header.stamp;
    response.timestamp_previous = previous_timestamp_;
    response.first_call = first_service_call_;
    // Store camera motion to response
    if (response.success)
    {
      // Left camera motion from previous frame to current frame
      Matrix camera_motion = visual_odometer_->getMotion();

      ROS_DEBUG("Found %i matches with %i inliers.", visual_odometer_->getNumberOfMatches(), visual_odometer_->getNumberOfInliers());
      ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << camera_motion);

      tf2::Matrix3x3 camera_rotation(camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
                              camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
                              camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
      tf2::Vector3 camera_translation(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
      tf2::Transform tf2_camera_motion(camera_rotation, camera_translation);

      sensor_frame_id_ = request.left_image.header.frame_id;
      integrateAndBroadcastTF(tf2_camera_motion, request.left_image.header.stamp);

      response.left_previous_to_current = tf2::toMsg(tf2_camera_motion);
    }

    if(!first_service_call_ && !response.success)
    {
      ROS_ERROR("VisualOdometryStereo::process() is failed");
    }

    ros::WallDuration runtime = ros::WallTime::now() - start_time;
    response.runtime.sec = runtime.sec;
    response.runtime.nsec = runtime.nsec;

    first_service_call_ = false;
    previous_timestamp_ = request.left_image.header.stamp;

    ROS_INFO("EstimateMotionFromStereo service is finished");

    return true;
  }

  void Viso2StereoServer::integrateAndBroadcastTF(const tf2::Transform& delta_transform, const ros::Time& timestamp)
  {
    integrated_pose_ *= delta_transform;

    // transform integrated pose to base frame
    std::string error_msg;
    tf2::Stamped<tf2::Transform> base_to_sensor;
    if (tf_buffer_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
    {
      geometry_msgs::TransformStamped base_to_sensor_msg;
      base_to_sensor_msg = tf_buffer_.lookupTransform(base_link_frame_id_, sensor_frame_id_, timestamp);
      tf2::fromMsg(base_to_sensor_msg, base_to_sensor);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                              "will assume it as identity!",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      base_to_sensor.setIdentity();
    }

    tf2::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();

    geometry_msgs::TransformStamped base_transform_msg = tf2::toMsg(tf2::Stamped<tf2::Transform>(base_transform, timestamp, odom_frame_id_));
    //geometry_msgs::TransformStamped base_transform_msg;
    base_transform_msg.child_frame_id = base_link_frame_id_;
    tf_broadcaster_.sendTransform(base_transform_msg);
  }

}

