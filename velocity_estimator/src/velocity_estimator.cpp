#include "velocity_estimator.h"
#include "velocity_estimator/pcl_point_xyz_velocity.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

VelocityEstimator::VelocityEstimator() {  
  reconfigure_func_ = boost::bind(&VelocityEstimator::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  pc_with_velocity_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("velocity_pc", 10);
  
  camera_transform_sub_.subscribe(node_handle_, "camera_transform", 1);
  optical_flow_left_sub_.subscribe(node_handle_, "optical_flow_left", 1); // optical flowはrectified imageで計算すること
  optical_flow_right_sub_.subscribe(node_handle_, "optical_flow_right", 1); // optical flowはrectified imageで計算すること
  disparity_image_sub_.subscribe(node_handle_, "disparity_image", 20);
  left_camera_info_sub_.subscribe(node_handle_, "left_camera_info", 1);

  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, optical_flow_msg::OpticalFlow, optical_flow_msg::OpticalFlow, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>>(camera_transform_sub_, optical_flow_left_sub_, optical_flow_right_sub_, left_camera_info_sub_, disparity_image_sub_, 50);
  time_sync_->registerCallback(boost::bind(&VelocityEstimator::dataCB, this, _1, _2, _3, _4, _5));
}

void VelocityEstimator::reconfigureCB(velocity_estimator::VelocityEstimatorConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: matching_tolerance = %f", config.matching_tolerance);
  
  matching_tolerance_ = config.matching_tolerance;
}

void VelocityEstimator::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_left, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  if (optical_flow_left->previous_stamp == time_stamp_previous_) {
    ros::Time start_process = ros::Time::now();

    cv::Mat left_flow = cv_bridge::toCvCopy(optical_flow_left->flow)->image;
    cv::Mat right_flow = cv_bridge::toCvCopy(optical_flow_right->flow)->image;
    
    DisparityImageProcessor disparity_processor(disparity_image, left_camera_info);
    DisparityImageProcessor disparity_processor_previous(disparity_image_previous_, left_camera_info);

    pcl::PointCloud<pcl::PointXYZ> pc_now, pc_previous;
    disparity_processor.toPointCloud(pc_now);
    disparity_processor_previous.toPointCloud(pc_previous);

    pcl::PointCloud<pcl::PointXYZ> pc_previous_transformed;
    transformPCPreviousToNow(pc_previous, pc_previous_transformed, camera_transform->transform);

    ros::Duration time_between_frames = camera_transform->header.stamp - time_stamp_previous_;

    pcl::PointXYZVelocity default_value;
    default_value.x = std::nanf("");
    default_value.y = std::nanf("");
    default_value.z = std::nanf("");
    default_value.vx = std::nanf("");
    default_value.vy = std::nanf("");
    default_value.vz = std::nanf("");
    pcl::PointCloud<pcl::PointXYZVelocity> pc_with_velocity(pc_now.width, pc_now.height, default_value);

    cv::Point2i left_now;
    for (left_now.y = 0; left_now.y < left_flow.rows; left_now.y++) 
    {
      for (left_now.x = 0; left_now.x < left_flow.cols; left_now.x++)
      {
        pcl::PointXYZVelocity &point_with_velocity = pc_with_velocity.at(left_now.x, left_now.y);
        pcl::PointXYZ point3d_now = pc_now.at(left_now.x, left_now.y);

        point_with_velocity.x = point3d_now.x;
        point_with_velocity.y = point3d_now.y;
        point_with_velocity.z = point3d_now.z;

        if (!isValid(point3d_now))
          continue;

        cv::Point2i left_previous, right_now, right_previous;
        
        if (!getPreviousPoint(left_now, left_previous, left_flow))
          continue;

        if (!getRightPoint(left_now, right_now, disparity_processor))
          continue;

        if (!getRightPoint(left_previous, right_previous, disparity_processor_previous))
          continue;
        
        if (right_now.x < 0 || right_now.x >= right_flow.cols)
          continue;
        
        cv::Vec2f flow_right = right_flow.at<cv::Vec2f>(right_now.y, right_now.x);
        
        if(std::isnan(flow_right[0]) || std::isnan(flow_right[1]))
          continue;

        if (matching_tolerance_ >= 0) { // matching_toleranceが負なら無効化
          double x_diff = right_previous.x + flow_right[0] - right_now.x;
          double y_diff = right_previous.y + flow_right[1] - right_now.y;
          double diff = std::sqrt(x_diff * x_diff + y_diff * y_diff);
          if (diff > matching_tolerance_)
            continue;
        }

        pcl::PointXYZ point3d_previous;
        point3d_previous = pc_previous_transformed.at(left_previous.x, left_previous.y);

        if (!isValid(point3d_previous))
          continue;      
        
        point_with_velocity.vx = (point3d_now.x - point3d_previous.x) / time_between_frames.toSec();
        point_with_velocity.vy = (point3d_now.y - point3d_previous.y) / time_between_frames.toSec();
        point_with_velocity.vz = (point3d_now.z - point3d_previous.z) / time_between_frames.toSec();
      }
    }
    
    sensor_msgs::PointCloud2 pc_with_velocity_msg;
    pcl::toROSMsg(pc_with_velocity, pc_with_velocity_msg);
    pc_with_velocity_msg.header.frame_id = left_camera_info->header.frame_id;
    pc_with_velocity_msg.header.stamp = left_camera_info->header.stamp;
    pc_with_velocity_pub_.publish(pc_with_velocity_msg);

    ros::Duration process_time = ros::Time::now() - start_process;
    ROS_INFO("process time: %f", process_time.toSec());
  }
  disparity_image_previous_ = disparity_image;
  time_stamp_previous_ = camera_transform->header.stamp;
}

void VelocityEstimator::transform(pcl::PointCloud<pcl::PointXYZ> &pc_in, pcl::PointCloud<pcl::PointXYZ> &pc_transformed, tf2::Transform transform)
{
  geometry_msgs::Transform msg_transform = tf2::toMsg(transform);
  this->transform(pc_in, pc_transformed, msg_transform);
}

void VelocityEstimator::transform(pcl::PointCloud<pcl::PointXYZ> &pc_in, pcl::PointCloud<pcl::PointXYZ> &pc_transformed, geometry_msgs::Transform transform)
{
  Eigen::Transform<float,3,Eigen::Affine> eigen_transform = Eigen::Translation3f(transform.translation.x, transform.translation.y, transform.translation.z) * Eigen::Quaternion<float>(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);

  pc_transformed = pcl::PointCloud<pcl::PointXYZ>(pc_in.width, pc_in.height);
  for (int u = 0; u < pc_in.width; u++) 
  {
    for (int v =0; v < pc_in.height; v++) 
    {
      pcl::PointXYZ &point = pc_in.at(u, v);
      Eigen::Vector3f eigen_transformed = eigen_transform * Eigen::Vector3f(point.x, point.y, point.z);
      pc_transformed.at(u, v) = pcl::PointXYZ(eigen_transformed.x(), eigen_transformed.y(), eigen_transformed.z());
    }
  }
}

void VelocityEstimator::transformPCPreviousToNow(pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &camera_now_to_previous)
{
  tf2::Transform tf2_now_to_previous;
  tf2::fromMsg(camera_now_to_previous, tf2_now_to_previous);
  transform(pc_previous, pc_previous_transformed, tf2_now_to_previous.inverse());
}


bool VelocityEstimator::isValid(const pcl::PointXYZ &point)
{
  if (std::isnan(point.x))
    return false;

  if (std::isinf(point.x))
    return false;
  
  return true;
}

bool VelocityEstimator::getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const cv::Mat &flow)
{
  cv::Vec2f flow_at_point;
  try {
    flow_at_point = flow.at<cv::Vec2f>(now.y, now.x);
  } catch (std::out_of_range e) {
    return false;
  }
        
  if (std::isnan(flow_at_point[0]) || std::isnan(flow_at_point[1]))
    return false;
  
  previous.x = std::round(now.x - flow_at_point[0]);
  previous.y = std::round(now.y - flow_at_point[1]);

  return true;
}

bool VelocityEstimator::getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor)
{
  float disparity;
  if (!disparity_processor.getDisparity(left.x, left.y, disparity))
    return false;
  if (std::isnan(disparity) || std::isinf(disparity) || disparity < 0)
    return false;

  right.x = std::round(left.x - disparity);
  right.y = left.y;

  return true;
}
