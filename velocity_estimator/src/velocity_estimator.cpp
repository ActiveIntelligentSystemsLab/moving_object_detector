#include "velocity_estimator.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <stdexcept>
#include <memory>

VelocityEstimator::VelocityEstimator() {
  image_transport.reset(new image_transport::ImageTransport(node_handle_));

  reconfigure_func_ = boost::bind(&VelocityEstimator::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  pc_with_velocity_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("velocity_pc", 10);
  velocity_image_pub_ = image_transport->advertise("velocity_image", 1);
  
  camera_transform_sub_.subscribe(node_handle_, "camera_transform", 1);
  optical_flow_left_sub_.subscribe(node_handle_, "optical_flow_left", 1); // optical flowはrectified imageで計算すること
  optical_flow_right_sub_.subscribe(node_handle_, "optical_flow_right", 1); // optical flowはrectified imageで計算すること
  disparity_image_sub_.subscribe(node_handle_, "disparity_image", 20);
  left_camera_info_sub_.subscribe(node_handle_, "left_camera_info", 1);

  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, optical_flow_msg::OpticalFlow, optical_flow_msg::OpticalFlow, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>>(camera_transform_sub_, optical_flow_left_sub_, optical_flow_right_sub_, left_camera_info_sub_, disparity_image_sub_, 50);
  time_sync_->registerCallback(boost::bind(&VelocityEstimator::dataCB, this, _1, _2, _3, _4, _5));
}

void VelocityEstimator::constructVelocityImage(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, cv::Mat &velocity_image)
{
  velocity_image = cv::Mat(velocity_pc.height, velocity_pc.width, CV_8UC3, cv::Vec3b(0, 0, 0));
  for (size_t i = 0; i < velocity_pc.size(); i++)
  {
    const pcl::PointXYZVelocity &velocity_point = velocity_pc.at(i);

    if (std::isnan(velocity_point.vx) || std::isinf(velocity_point.vx))
      continue;

    double red, green, blue;

    // Regularize to [0.0, 1.0]
    red = std::abs(velocity_point.vx) / max_color_velocity_;
    red = std::min(red, 1.0);

    green = std::abs(velocity_point.vy) / max_color_velocity_;
    green = std::min(green, 1.0);

    blue = std::abs(velocity_point.vz) / max_color_velocity_;
    blue = std::min(blue, 1.0);

    velocity_image.at<cv::Vec3b>(i) = cv::Vec3b(blue * 255, green * 255, red * 255);
  }
}

void VelocityEstimator::constructVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
{
  pcl::PointCloud<pcl::PointXYZ> pc_previous_transformed;
  transformPCPreviousToNow(*pc_previous_, pc_previous_transformed, transform_now_to_previous_.transform);

  ros::Duration time_between_frames = transform_now_to_previous_.header.stamp - time_stamp_previous_;

  initializeVelocityPC(velocity_pc);

  cv::Point2i left_now;
  for (left_now.y = 0; left_now.y < left_flow_.rows; left_now.y++)
  {
    for (left_now.x = 0; left_now.x < left_flow_.cols; left_now.x++)
    {
      pcl::PointXYZVelocity &point_with_velocity = velocity_pc.at(left_now.x, left_now.y);
      pcl::PointXYZ point3d_now = pc_now_->at(left_now.x, left_now.y);

      if (!isValid(point3d_now))
        continue;

      point_with_velocity.x = point3d_now.x;
      point_with_velocity.y = point3d_now.y;
      point_with_velocity.z = point3d_now.z;

      cv::Point2i left_previous, right_now, right_previous;

      if (!getMatchPoints(left_now, left_previous, right_now, right_previous))
        continue;

      pcl::PointXYZ point3d_previous;
      point3d_previous = pc_previous_transformed.at(left_previous.x, left_previous.y);
      if (!isValid(point3d_previous))
        continue;

      point_with_velocity.vx = (point3d_now.x - point3d_previous.x) / time_between_frames.toSec();
      point_with_velocity.vy = (point3d_now.y - point3d_previous.y) / time_between_frames.toSec();
      point_with_velocity.vz = (point3d_now.z - point3d_previous.z) / time_between_frames.toSec();
    }
  }
}

void VelocityEstimator::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_left, const optical_flow_msg::OpticalFlowConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  disparity_now_.reset(new DisparityImageProcessor(disparity_image, left_camera_info));
  pc_now_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  disparity_now_->toPointCloud(*pc_now_);

  if (optical_flow_left->previous_stamp == time_stamp_previous_) {
    ros::Time start_process = ros::Time::now();

    left_flow_ = cv_bridge::toCvCopy(optical_flow_left->flow)->image;
    right_flow_ = cv_bridge::toCvCopy(optical_flow_right->flow)->image;

    transform_now_to_previous_ = *camera_transform;

    pcl::PointCloud<pcl::PointXYZVelocity> pc_with_velocity;
    constructVelocityPC(pc_with_velocity);
    
    publishPointcloud(pc_with_velocity, left_camera_info->header.frame_id, left_camera_info->header.stamp);

    if (velocity_image_pub_.getNumSubscribers() > 0)
    {
      cv::Mat velocity_image;
      constructVelocityImage(pc_with_velocity, velocity_image);
      publishVelocityImage(velocity_image);
    }

    ros::Duration process_time = ros::Time::now() - start_process;
    ROS_INFO("process time: %f", process_time.toSec());
  }

  disparity_previous_ = disparity_now_;
  pc_previous_ = pc_now_;
  time_stamp_previous_ = transform_now_to_previous_.header.stamp;
}

bool VelocityEstimator::getMatchPoints(const cv::Point2i &left_now, cv::Point2i &left_previous, cv::Point2i &right_now, cv::Point2i &right_previous)
{
  if (!getPreviousPoint(left_now, left_previous, left_flow_))
    return false;

  if (!getRightPoint(left_now, right_now, *disparity_now_))
    return false;

  if (!getRightPoint(left_previous, right_previous, *disparity_previous_))
    return false;

  if (right_now.x < 0 || right_now.x >= right_flow_.cols)
    return false;

  cv::Vec2f flow_right = right_flow_.at<cv::Vec2f>(right_now.y, right_now.x);

  if(std::isnan(flow_right[0]) || std::isnan(flow_right[1]))
    return false;

  if (matching_tolerance_ >= 0) { // matching_toleranceが負なら無効化
    double x_diff = right_previous.x + flow_right[0] - right_now.x;
    double y_diff = right_previous.y + flow_right[1] - right_now.y;
    double diff = std::sqrt(x_diff * x_diff + y_diff * y_diff);
    if (diff > matching_tolerance_)
      return false;
  }

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

/**
 * \brief Resize velocity pointcloud and fill each point by default value
 *
 * \param velocity_pc Target velocity pointcloud
 */
void VelocityEstimator::initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
{
  pcl::PointXYZVelocity default_value;
  default_value.x = std::nanf("");
  default_value.y = std::nanf("");
  default_value.z = std::nanf("");
  default_value.vx = std::nanf("");
  default_value.vy = std::nanf("");
  default_value.vz = std::nanf("");
  velocity_pc = pcl::PointCloud<pcl::PointXYZVelocity>(pc_now_->width, pc_now_->height, default_value);
}

bool VelocityEstimator::isValid(const pcl::PointXYZ &point)
{
  if (std::isnan(point.x))
    return false;

  if (std::isinf(point.x))
    return false;
  
  return true;
}

void VelocityEstimator::publishVelocityImage(const cv::Mat &velocity_image)
{
  std_msgs::Header header;
  header.frame_id = transform_now_to_previous_.header.frame_id;
  header.stamp = transform_now_to_previous_.header.stamp;

  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, velocity_image);
  velocity_image_pub_.publish(cv_image.toImageMsg());
}

template <typename PointT> void VelocityEstimator::publishPointcloud(const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  pc_with_velocity_pub_.publish(pointcloud_msg);
}

void VelocityEstimator::reconfigureCB(velocity_estimator::VelocityEstimatorConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: matching_tolerance = %f", config.matching_tolerance);

  matching_tolerance_ = config.matching_tolerance;
}

void VelocityEstimator::transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &now_to_previous)
{
  Eigen::Isometry3d eigen_now_to_previous = tf2::transformToEigen(now_to_previous);
  Eigen::Isometry3d eigen_previous_to_now = eigen_now_to_previous.inverse();

  pc_previous_transformed = pcl::PointCloud<pcl::PointXYZ>(pc_previous.width, pc_previous.height);
  for (int u = 0; u < pc_previous.width; u++)
  {
    for (int v =0; v < pc_previous.height; v++)
    {
      const pcl::PointXYZ &point = pc_previous.at(u, v);
      Eigen::Vector3d eigen_transformed = eigen_previous_to_now * Eigen::Vector3d(point.x, point.y, point.z);
      pc_previous_transformed.at(u, v) = pcl::PointXYZ(eigen_transformed.x(), eigen_transformed.y(), eigen_transformed.z());
    }
  }
}
