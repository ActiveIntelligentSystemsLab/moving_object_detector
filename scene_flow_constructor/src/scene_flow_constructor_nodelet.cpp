#include <pluginlib/class_list_macros.h>

#include "scene_flow_constructor_nodelet.h"

PLUGINLIB_EXPORT_CLASS(scene_flow_constructor::SceneFlowConstructorNodelet, nodelet::Nodelet)

// ROS headers
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_common.h>
#include <optical_flow_srvs/CalculateDenseOpticalFlow.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sgm_gpu/EstimateDisparity.h>
#include <tf2_eigen/tf2_eigen.h>
#include <viso2_stereo_server/EstimateMotionFromStereo.h>

// Non-ROS headers
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <memory>

namespace scene_flow_constructor {

void SceneFlowConstructorNodelet::onInit() {
  ros::NodeHandle &node_handle = getNodeHandle();
  ros::NodeHandle &private_node_handle = getPrivateNodeHandle();

  // Prepare service clients
  motion_service_client_ = node_handle.serviceClient<viso2_stereo_server::EstimateMotionFromStereo>("estimate_motion_from_stereo");
  motion_service_client_.waitForExistence();

  disparity_service_client_ = node_handle.serviceClient<sgm_gpu::EstimateDisparity>("estimate_disparity");
  disparity_service_client_.waitForExistence();

  optflow_service_client_ = node_handle.serviceClient<optical_flow_srvs::CalculateDenseOpticalFlow>("calculate_dense_optical_flow");
  optflow_service_client_.waitForExistence();

  // Dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(private_node_handle));
  reconfigure_func_ = boost::bind(&SceneFlowConstructorNodelet::reconfigureCB, this, _1, _2);
  reconfigure_server_->setCallback(reconfigure_func_);
  
  // Publishers
  image_transport_.reset(new image_transport::ImageTransport(private_node_handle));
  pc_with_velocity_pub_ = private_node_handle.advertise<sensor_msgs::PointCloud2>("scene_flow", 10);
  static_flow_pub_ = private_node_handle.advertise<optical_flow_msgs::DenseOpticalFlow>("synthetic_optical_flow", 1);
  velocity_image_pub_ = image_transport_->advertise("scene_flow_image", 1);
  flow_residual_pub_ = image_transport_->advertise("optical_flow_residual", 1);

  // Subscribers
  std::string left_image_topic = node_handle.resolveName("left_image");
  std::string right_image_topic = node_handle.resolveName("right_image");
  std::string left_caminfo_topic = image_transport::getCameraInfoTopic(left_image_topic);
  std::string right_caminfo_topic = image_transport::getCameraInfoTopic(right_image_topic);
  left_image_sub_.subscribe(*image_transport, left_image_topic, 10);
  right_image_sub_.subscribe(*image_transport, right_image_topic, 10);
  left_caminfo_sub_.subscribe(node_handle, left_caminfo_topic, 10);
  right_caminfo_sub_.subscribe(node_handle, right_caminfo_topic, 10);

  // Stereo synchronizer
  stereo_synchronizer_.reset(new StereoSynchronizer(left_image_sub_, right_image_sub_, left_caminfo_sub_, right_caminfo_sub_, 10));
  stereo_synchronizer_->registerCallback(&SceneFlowConstructorNodelet::stereoCallback, this);
}

void SceneFlowConstructorNodelet::calculateStaticOpticalFlow()
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

void SceneFlowConstructorNodelet::constructVelocityImage(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, cv::Mat &velocity_image)
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

void SceneFlowConstructorNodelet::constructVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
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

void estimateCameraMotion(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::ImageConstPtr& right_camera_info)
{
  viso2_stereo_server::EstimateMotionFromStereo::Request request;
  request.left_image = *left_image;
  request.right_image = *right_image;
  request.right_camera_info = *right_camera_info;
  request.left_camera_infa = *left_camera_info;

  viso2_stereo_servea::EstimateMotionFromStereo::Response response;
  bool success = motion_service_client_.call(request, response);

  if (success && !motion_response.first_call && motion_response.success)
  {
    transform_prev2now_.reset(new geometry_msgs::Transform());
    *transform_prev2now_ = motion_response.left_previous_to_current;
  }
  else if(!motion_response.first_call)
  {
    ROS_ERROR_STREAM("Visual odometry is failed\nInput timestamp: " << *left_image.header.stamp);
    transform_prev2now_.reset();
  }
}

void estimateDisparity(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::ImageConstPtr& right_camera_info)
{
  sgm_gpu::EstimateDisparity::Request request;
  request.left_image = *left_image;
  request.right_image = *right_image;
  request.left_camera_info = *left_camera_info;
  request.right_camera_info = *right_camera_info;
  sgm_gpu::EstimateDisparity::Request response;
  bool success = disparity_service_client_.call(request, response);
  disparity_previous_ = disparity_now_;
  if (disparity_success)
    disparity_now_.reset(new DisparityImageProcessor(disparity_response, *left_camera_info));
  else
  {
    ROS_ERROR_STREAM("Disparity estimation is failed\nInput timestamp: " << *left_image.header.stamp);
    disparity_now_.reset();
  }
}

void estimateOpticalFlow(const sensor_msgs::ImageConstPtr& left_image)
{
  optical_flow_srvs::CalculateDenseOpticalFlow::Request request;
  request.older_image = previous_left_image_;
  request.newer_image = *left_image;

  optical_flow_srvs::CalculateDenseOpticalFlow::Response optflow_response;
  bool success = optflow_service_client_.call(request, response);

  if (success)
  {
    _left_flow.reset(new optical_flow_msgs::DenseOpticalFlow());
    *_left_flow = optflow_response.optical_flow;
  }
  else
  {
    ROS_ERROR_STREAM("Optical flow estimation is failed\nInput timestamp: " << *left_image.header.stamp);
    _left_flow.reset();
  }
}
  
bool SceneFlowConstructorNodelet::getMatchPoints(const cv::Point2i &left_now, cv::Point2i &left_previous, cv::Point2i &right_now, cv::Point2i &right_previous)
{
  if (!getPreviousPoint(left_now, left_previous, *left_flow_))
    return false;

  if (!getRightPoint(left_now, right_now, *disparity_now_))
    return false;

  if (!getRightPoint(left_previous, right_previous, *disparity_previous_))
    return false;

  return true;
}

bool SceneFlowConstructorNodelet::getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const optical_flow_msgs::DenseOpticalFlow &flow)
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

bool SceneFlowConstructorNodelet::getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor)
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

void SceneFlowConstructorNodelet::initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
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

bool SceneFlowConstructorNodelet::isValid(const pcl::PointXYZ &point)
{
  if (std::isnan(point.x))
    return false;

  if (std::isinf(point.x))
    return false;
  
  return true;
}

void SceneFlowConstructorNodelet::publishFlowResidual()
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

void SceneFlowConstructorNodelet::publishStaticOpticalFlow()
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

void SceneFlowConstructorNodelet::publishVelocityImage(const cv::Mat &velocity_image)
{
  std_msgs::Header header;
  header.frame_id = transform_now_to_previous_.header.frame_id;
  header.stamp = transform_now_to_previous_.header.stamp;

  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, velocity_image);
  velocity_image_pub_.publish(cv_image.toImageMsg());
}

template <typename PointT> void SceneFlowConstructorNodelet::publishPointcloud(const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  pc_with_velocity_pub_.publish(pointcloud_msg);
}

void stereoCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::ImageConstPtr& right_camera_info)
{
  ros::Time start_process = ros::WallTime::now();

  // Get disparity, optical flow and camera motion by calling external services
  estimateDisparity(left_image, right_image, left_camera_info, right_camera_info);
  if (previous_left_image_)
    estimateOpticalFlow(left_image);
  estimateCameraMotion(left_image, right_image, left_camera_info, right_camera_info);

  // Set params
  left_cam_model_.fromCameraInfo(left_camera_info);
  time_stamp_previous_ = time_stamp_now_;
  time_stamp_now_ = left_image->header.stamp;
  camera_frame_id_ = left_image->header.frame_id;
  image_width_ = left_image->width;
  image_height_ = left_image->height;

  // Construct pointcloud from disparity for now frame
  pc_previous_ = pc_now_;
  if (disparity_now_)
  {
    pc_now_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    disparity_now_->toPointCloud(*pc_now_);
  }
  else
    pc_now_.reset();

  // Transform previous pointcloud by estimated camera motion
  if (pc_previous_ && transform_prev2now_)
  {
    pc_previous_transformed_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    transformPCPreviousToNow(*pc_previous_, *pc_previous_transformed_, *transform_prev2now);
  }
  else
    pc_previous_transformed_.reset();

  if (pc_now && pc_previous_transformed_) 
  {
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
  }
  ros::WallDuration process_time = ros::WallTime::now() - start_process;
  NODELET_INFO("process time: %f", process_time.toSec());

  // Save images
  previous_left_image_ = left_image;
  previous_right_image_ = right_image;
}

void SceneFlowConstructorNodelet::reconfigureCB(scene_flow_constructor::SceneFlowConstructorConfig& config, uint32_t level)
{
  NODELET_INFO("Reconfigure Request: dynamic_flow_diff = %d, max_color_velocity = %f", config.dynamic_flow_diff, config.max_color_velocity);

  dynamic_flow_diff_  = config.dynamic_flow_diff;
  max_color_velocity_ = config.max_color_velocity;
}

void SceneFlowConstructorNodelet::transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &previous_to_now)
{
  Eigen::Isometry3d eigen_prev2now = tf2::transformToEigen(previous_to_now);

  pc_previous_transformed = pcl::PointCloud<pcl::PointXYZ>(image_width_, image_height_);
  for (int u = 0; u < image_width_; u++)
  {
    for (int v = 0; v < image_height_; v++)
    {
      const pcl::PointXYZ &point = pc_previous.at(u, v);
      Eigen::Vector3d eigen_transformed = eigen_prev2now * Eigen::Vector3d(point.x, point.y, point.z);
      pc_previous_transformed.at(u, v) = pcl::PointXYZ(eigen_transformed.x(), eigen_transformed.y(), eigen_transformed.z());
    }
  }
}

} // namespace scene_flow_constructor
