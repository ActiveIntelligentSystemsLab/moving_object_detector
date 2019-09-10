#include <pluginlib/class_list_macros.h>

#include "velocity_estimator_nodelet.h"

PLUGINLIB_EXPORT_CLASS(velocity_estimator::VelocityEstimatorNodelet, nodelet::Nodelet)

// ROS headers
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

// Non-ROS headers
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <memory>

namespace velocity_estimator{

void VelocityEstimatorNodelet::onInit() {
  ros::NodeHandle &node_handle = getNodeHandle();
  ros::NodeHandle &private_node_handle = getPrivateNodeHandle();

  image_transport_.reset(new image_transport::ImageTransport(private_node_handle));

  reconfigure_server_.reset(new ReconfigureServer(private_node_handle));
  reconfigure_func_ = boost::bind(&VelocityEstimatorNodelet::reconfigureCB, this, _1, _2);
  reconfigure_server_->setCallback(reconfigure_func_);
  
  pc_with_velocity_pub_ = private_node_handle.advertise<sensor_msgs::PointCloud2>("velocity_pc", 10);
  static_flow_pub_ = private_node_handle.advertise<optical_flow_msgs::DenseOpticalFlow>("static_flow", 1);
  velocity_image_pub_ = image_transport_->advertise("velocity_image", 1);
  flow_residual_pub_ = image_transport_->advertise("flow_residual", 1);
  
  camera_transform_sub_.subscribe(node_handle, "camera_transform", 1);
  optical_flow_left_sub_.subscribe(node_handle, "optical_flow_left", 1); // optical flowはrectified imageで計算すること
  disparity_image_sub_.subscribe(node_handle, "disparity_image", 20);
  left_camera_info_sub_.subscribe(node_handle, "left_camera_info", 1);

  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, optical_flow_msgs::DenseOpticalFlow, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>>(camera_transform_sub_, optical_flow_left_sub_, left_camera_info_sub_, disparity_image_sub_, 50);
  time_sync_->registerCallback(boost::bind(&VelocityEstimatorNodelet::dataCB, this, _1, _2, _3, _4));
}

void VelocityEstimatorNodelet::calculateStaticOpticalFlow()
{
  left_static_flow_ = cv::Mat(image_height_, image_width_, CV_32FC2);

  for (int y = 0; y < image_height_; y++)
  {
    for (int x = 0; x < image_width_; x++)
    {
      pcl::PointXYZ pcl_point = pc_previous_transformed_->at(x, y);
      cv::Point3d point_3d;
      point_3d.x = pcl_point.x;
      point_3d.y = pcl_point.y;
      point_3d.z = pcl_point.z;
      cv::Point2d point_2d = left_cam_model_.project3dToPixel(point_3d);
      cv::Vec2f static_flow(point_2d.x, point_2d.y);
      left_static_flow_.at<cv::Vec2f>(y, x) = cv::Vec2f(point_2d.x - x, point_2d.y - y);
    }
  }
}

void VelocityEstimatorNodelet::checkSameFrameId(const geometry_msgs::TransformStampedConstPtr& camera_transform, const optical_flow_msgs::DenseOpticalFlowConstPtr& left_optical_flow, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  if (camera_transform->header.frame_id == left_optical_flow->header.frame_id &&
      camera_transform->header.frame_id == left_camera_info->header.frame_id &&
      camera_transform->header.frame_id == disparity_image->header.frame_id)
    return;
  else
  {
    ROS_FATAL_STREAM("Frame id of synchronized messages are not same!\n" <<
                     "camera transform : " << camera_transform->header.frame_id  << "\n" <<
                     "optical flow : "     << left_optical_flow->header.frame_id << "\n" <<
                     "camera info : "      << left_camera_info->header.frame_id  << "\n" <<
                     "disparity image : "  << disparity_image->header.frame_id);
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }
}

void VelocityEstimatorNodelet::checkSameSize(const optical_flow_msgs::DenseOpticalFlowConstPtr& left_optical_flow, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  if (left_optical_flow->width == left_camera_info->width && left_optical_flow->height == left_camera_info->height &&
      left_optical_flow->width == disparity_image->image.width && left_optical_flow->height == disparity_image->image.width)
    return;
  else
  {
    ROS_FATAL_STREAM("Image size of synchronized messages are not same!\n" <<
                     "optical flow : "    << left_optical_flow->width     << "x" << left_optical_flow->height << "\n" <<
                     "camera info : "     << left_camera_info->width      << "x" << left_camera_info->height  << "\n" <<
                     "disparity image : " << disparity_image->image.width << "x" << disparity_image->image.height);
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }
}

void VelocityEstimatorNodelet::constructVelocityImage(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, cv::Mat &velocity_image)
{
  velocity_image = cv::Mat(image_height_, image_width_, CV_8UC3, cv::Vec3b(0, 0, 0));
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

void VelocityEstimatorNodelet::constructVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
{
  ros::Duration time_between_frames = transform_now_to_previous_.header.stamp - time_stamp_previous_;

  initializeVelocityPC(velocity_pc);

  cv::Point2i left_now;
  for (left_now.y = 0; left_now.y < image_height_; left_now.y++)
  {
    for (left_now.x = 0; left_now.x < image_width_; left_now.x++)
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
      point3d_previous = pc_previous_transformed_->at(left_previous.x, left_previous.y);
      if (!isValid(point3d_previous))
        continue;

      cv::Vec2f flow;
      flow[0] = left_flow_->flow_field[left_now.y * image_width_ + left_now.x].x;
      flow[1] = left_flow_->flow_field[left_now.y * image_width_ + left_now.x].y;
      cv::Vec2f static_flow = left_static_flow_.at<cv::Vec2f>(left_now.y, left_now.x);

      cv::Vec2f flow_diff = flow - static_flow;

      if (std::sqrt(flow_diff.dot(flow_diff)) >= dynamic_flow_diff_)
      {
        point_with_velocity.vx = (point3d_now.x - point3d_previous.x) / time_between_frames.toSec();
        point_with_velocity.vy = (point3d_now.y - point3d_previous.y) / time_between_frames.toSec();
        point_with_velocity.vz = (point3d_now.z - point3d_previous.z) / time_between_frames.toSec();
      }
      else
      {
        point_with_velocity.vx = 0.0;
        point_with_velocity.vy = 0.0;
        point_with_velocity.vz = 0.0;
      }
    }
  }
}

void VelocityEstimatorNodelet::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const optical_flow_msgs::DenseOpticalFlowConstPtr& optical_flow_left, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  checkSameFrameId(camera_transform, optical_flow_left, left_camera_info, disparity_image);
  checkSameSize(optical_flow_left, left_camera_info, disparity_image);

  disparity_now_.reset(new DisparityImageProcessor(disparity_image, left_camera_info));
  pc_now_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  disparity_now_->toPointCloud(*pc_now_);

  transform_now_to_previous_ = *camera_transform;

  left_cam_model_.fromCameraInfo(left_camera_info);

  left_flow_ = optical_flow_left;

  time_stamp_now_ = camera_transform->header.stamp;
  camera_frame_id_ = left_camera_info->header.frame_id;
  image_width_ = optical_flow_left->width;
  image_height_ = optical_flow_left->height;

  if (optical_flow_left->previous_stamp == time_stamp_previous_) {
    ros::Time start_process = ros::Time::now();

    pc_previous_transformed_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    transformPCPreviousToNow(*pc_previous_, *pc_previous_transformed_, transform_now_to_previous_.transform);

    calculateStaticOpticalFlow();

    pcl::PointCloud<pcl::PointXYZVelocity> pc_with_velocity;
    constructVelocityPC(pc_with_velocity);
    
    if (pc_with_velocity_pub_.getNumSubscribers() > 0)
      publishPointcloud(pc_with_velocity, left_camera_info->header.frame_id, left_camera_info->header.stamp);


    if (velocity_image_pub_.getNumSubscribers() > 0)
    {
      cv::Mat velocity_image;
      constructVelocityImage(pc_with_velocity, velocity_image);
      publishVelocityImage(velocity_image);
    }

    if (static_flow_pub_.getNumSubscribers() > 0)
      publishStaticOpticalFlow();

    if (flow_residual_pub_.getNumSubscribers() > 0)
      publishFlowResidual();

    ros::Duration process_time = ros::Time::now() - start_process;
    NODELET_INFO("process time: %f", process_time.toSec());
  }

  disparity_previous_ = disparity_now_;
  pc_previous_ = pc_now_;
  time_stamp_previous_ = time_stamp_now_;
}

bool VelocityEstimatorNodelet::getMatchPoints(const cv::Point2i &left_now, cv::Point2i &left_previous, cv::Point2i &right_now, cv::Point2i &right_previous)
{
  if (!getPreviousPoint(left_now, left_previous, *left_flow_))
    return false;

  if (!getRightPoint(left_now, right_now, *disparity_now_))
    return false;

  if (!getRightPoint(left_previous, right_previous, *disparity_previous_))
    return false;

  return true;
}

bool VelocityEstimatorNodelet::getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const optical_flow_msgs::DenseOpticalFlow &flow)
{
  if (now.x < 0 || now.x >= image_width_ || now.y < 0 || now.y >= image_height_) {
    return false;
  }

  int flow_index_now = now.y * image_width_ + now.x;
  if (flow.invalid_map[flow_index_now])
    return false;
  
  optical_flow_msgs::PixelDisplacement flow_at_point = flow.flow_field[flow_index_now];

  if (std::isnan(flow_at_point.x) || std::isnan(flow_at_point.y))
    return false;

  previous.x = std::round(now.x - flow_at_point.x);
  previous.y = std::round(now.y - flow_at_point.y);

  return true;
}

bool VelocityEstimatorNodelet::getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor)
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

void VelocityEstimatorNodelet::initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
{
  pcl::PointXYZVelocity default_value;
  default_value.x = std::nanf("");
  default_value.y = std::nanf("");
  default_value.z = std::nanf("");
  default_value.vx = std::nanf("");
  default_value.vy = std::nanf("");
  default_value.vz = std::nanf("");
  velocity_pc = pcl::PointCloud<pcl::PointXYZVelocity>(image_width_, image_height_, default_value);
}

bool VelocityEstimatorNodelet::isValid(const pcl::PointXYZ &point)
{
  if (std::isnan(point.x))
    return false;

  if (std::isinf(point.x))
    return false;
  
  return true;
}

void VelocityEstimatorNodelet::publishFlowResidual()
{
  std_msgs::Header header;
  header.frame_id = camera_frame_id_;
  header.stamp = time_stamp_now_;

  cv_bridge::CvImage flow_residual(header, "mono8");
  flow_residual.image = cv::Mat(image_height_, image_width_, CV_8UC1);
  
  int total_pixel = image_height_ * image_width_;
  for (int pixel_index = 0; pixel_index < total_pixel; pixel_index++) 
  {
    optical_flow_msgs::PixelDisplacement flow_pixel = left_flow_->flow_field[pixel_index];
    cv::Vec2f& static_flow_pixel = left_static_flow_.at<cv::Vec2f>(pixel_index);

    float residual_pixel = std::sqrt(std::pow(flow_pixel.x - static_flow_pixel[0], 2) + std::pow(flow_pixel.y - static_flow_pixel[1], 2));
    residual_pixel = std::min(residual_pixel, 255.0f);
    flow_residual.image.at<unsigned char>(pixel_index) = static_cast<unsigned char>(residual_pixel);
  }

  sensor_msgs::ImagePtr flow_residual_msg = flow_residual.toImageMsg();
  flow_residual_pub_.publish(flow_residual_msg);
}

void VelocityEstimatorNodelet::publishStaticOpticalFlow()
{
  std_msgs::Header header;
  header.frame_id = transform_now_to_previous_.header.frame_id;
  header.stamp = transform_now_to_previous_.header.stamp;

  optical_flow_msgs::DenseOpticalFlow flow_msg;
  flow_msg.header = header;
  flow_msg.previous_stamp = time_stamp_previous_;
  flow_msg.width = left_static_flow_.cols;
  flow_msg.height = left_static_flow_.rows;
  flow_msg.flow_field.resize(image_width_ * image_height_);
  flow_msg.invalid_map.resize(image_width_ * image_height_, false);

  for (int y = 0; y < image_height_; y++) 
  {
    for (int x = 0; x < image_width_; x++)
    {
      cv::Vec2f& flow_at_point = left_static_flow_.at<cv::Vec2f>(y, x);
      flow_msg.flow_field[y * flow_msg.width + x].x = flow_at_point[0];
      flow_msg.flow_field[y * flow_msg.width + x].y = flow_at_point[1];
    }
  }

  static_flow_pub_.publish(flow_msg);
}

void VelocityEstimatorNodelet::publishVelocityImage(const cv::Mat &velocity_image)
{
  std_msgs::Header header;
  header.frame_id = transform_now_to_previous_.header.frame_id;
  header.stamp = transform_now_to_previous_.header.stamp;

  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, velocity_image);
  velocity_image_pub_.publish(cv_image.toImageMsg());
}

template <typename PointT> void VelocityEstimatorNodelet::publishPointcloud(const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  pc_with_velocity_pub_.publish(pointcloud_msg);
}

void VelocityEstimatorNodelet::reconfigureCB(velocity_estimator::VelocityEstimatorConfig& config, uint32_t level)
{
  NODELET_INFO("Reconfigure Request: dynamic_flow_diff = %d, max_color_velocity = %f", config.dynamic_flow_diff, config.max_color_velocity);

  dynamic_flow_diff_  = config.dynamic_flow_diff;
  max_color_velocity_ = config.max_color_velocity;
}

void VelocityEstimatorNodelet::transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &now_to_previous)
{
  Eigen::Isometry3d eigen_now_to_previous = tf2::transformToEigen(now_to_previous);
  Eigen::Isometry3d eigen_previous_to_now = eigen_now_to_previous.inverse();

  pc_previous_transformed = pcl::PointCloud<pcl::PointXYZ>(image_width_, image_height_);
  for (int u = 0; u < image_width_; u++)
  {
    for (int v =0; v < image_height_; v++)
    {
      const pcl::PointXYZ &point = pc_previous.at(u, v);
      Eigen::Vector3d eigen_transformed = eigen_previous_to_now * Eigen::Vector3d(point.x, point.y, point.z);
      pc_previous_transformed.at(u, v) = pcl::PointXYZ(eigen_transformed.x(), eigen_transformed.y(), eigen_transformed.z());
    }
  }
}

} // namespace velocity_estimator
