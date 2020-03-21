#include "scene_flow_constructor.h"
#include "odometry_params.h"

// ROS headers
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/camera_common.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Non-ROS headers
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>

namespace scene_flow_constructor {

SceneFlowConstructor::SceneFlowConstructor() {
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  // Load parameters for visual odometry
  ros::NodeHandle visual_odometry_nh(private_node_handle, "visual_odometry");
  odometry_params::loadParams(visual_odometry_nh, visual_odometer_params_);
  visual_odometry_nh.param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
  visual_odometry_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));

  integrated_pose_.setIdentity();
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  sgm_gpu_.reset(new sgm_gpu::SgmGpu(private_node_handle));

  image_transport_.reset(new image_transport::ImageTransport(private_node_handle));

  // Dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(private_node_handle));
  reconfigure_func_ = boost::bind(&SceneFlowConstructor::reconfigureCB, this, _1, _2);
  reconfigure_server_->setCallback(reconfigure_func_);
  
  // Publishers
  depth_pub_ = private_node_handle.advertise<sensor_msgs::Image>("depth", 1);
  optflow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("optical_flow", 1);
  pc_with_velocity_pub_ = private_node_handle.advertise<sensor_msgs::PointCloud2>("scene_flow", 1);
  static_flow_pub_ = private_node_handle.advertise<sensor_msgs::Image>("synthetic_optical_flow", 1);

  // Subscribers
  std::string left_image_topic = node_handle.resolveName("left_image");
  std::string right_image_topic = node_handle.resolveName("right_image");
  std::string left_caminfo_topic = image_transport::getCameraInfoTopic(left_image_topic);
  std::string right_caminfo_topic = image_transport::getCameraInfoTopic(right_image_topic);
  left_image_sub_.subscribe(*image_transport_, left_image_topic, 1);
  right_image_sub_.subscribe(*image_transport_, right_image_topic, 1);
  left_caminfo_sub_.subscribe(node_handle, left_caminfo_topic, 1);
  right_caminfo_sub_.subscribe(node_handle, right_caminfo_topic, 1);

  // Stereo synchronizer
  stereo_synchronizer_.reset(new StereoSynchronizer(left_image_sub_, right_image_sub_, left_caminfo_sub_, right_caminfo_sub_, 1));
  stereo_synchronizer_->registerCallback(&SceneFlowConstructor::stereoCallback, this);
}

void SceneFlowConstructor::calculateStaticOpticalFlow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, cv::Mat &left_static_flow)
{
  left_static_flow = cv::Mat(image_height_, image_width_, CV_32FC2);

  for (int y = 0; y < image_height_; y++)
  {
    for (int x = 0; x < image_width_; x++)
    {
      pcl::PointXYZ pcl_point = pc_previous_transformed.at(x, y);
      if (std::isnan(pcl_point.x))
      {
        left_static_flow.at<cv::Vec2f>(y, x) = cv::Vec2f(std::nanf(""), std::nanf(""));
        continue;
      }

      cv::Point3d point_3d;
      point_3d.x = pcl_point.x;
      point_3d.y = pcl_point.y;
      point_3d.z = pcl_point.z;
      cv::Point2d point_2d = left_cam_model_->project3dToPixel(point_3d);
      cv::Vec2f static_flow(point_2d.x, point_2d.y);
      left_static_flow.at<cv::Vec2f>(y, x) = cv::Vec2f(point_2d.x - x, point_2d.y - y);
    }
  }
}

void SceneFlowConstructor::construct
(
  std::shared_ptr<DisparityImageProcessor> disparity_now,
  std::shared_ptr<DisparityImageProcessor> disparity_previous,
  std::shared_ptr<cv_bridge::CvImage> left_flow,
  geometry_msgs::TransformPtr transform_prev2now
)
{
  if (left_flow && optflow_pub_.getNumSubscribers() > 0)
    optflow_pub_.publish(left_flow->toImageMsg());

  // Construct pointcloud from disparity for now frame
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_now, pc_previous;
  if (disparity_previous)
  {
    pc_previous.reset(new pcl::PointCloud<pcl::PointXYZ>());
    disparity_previous->toPointCloud(*pc_previous);
  }

  if (disparity_now)
  {
    pc_now.reset(new pcl::PointCloud<pcl::PointXYZ>());
    disparity_now->toPointCloud(*pc_now);
    if (depth_pub_.getNumSubscribers() > 0)
    {
      cv::Mat depth_now;
      disparity_now->toDepthImage(depth_now);
      publishDepthImage(depth_pub_, depth_now, disparity_now->_disparity_msg.header.stamp);
    }
  }

  if (!left_flow)
    return;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_previous_transformed;
  // Transform previous pointcloud by estimated camera motion
  if (pc_previous && transform_prev2now)
  {
    pc_previous_transformed.reset(new pcl::PointCloud<pcl::PointXYZ>());
    transformPCPreviousToNow(*pc_previous, *pc_previous_transformed, *transform_prev2now);
  }

  if (pc_now && pc_previous_transformed) 
  {
    cv_bridge::CvImage left_static_flow(left_flow->header, sensor_msgs::image_encodings::TYPE_32FC2);
    calculateStaticOpticalFlow(*pc_previous_transformed, left_static_flow.image);

    pcl::PointCloud<pcl::PointXYZVelocity> pc_with_velocity;
    constructVelocityPC(*pc_now, *pc_previous_transformed, *left_flow, left_static_flow.image, *disparity_now, *disparity_previous, pc_with_velocity);

    if (pc_with_velocity_pub_.getNumSubscribers() > 0)
      publishPointcloud(pc_with_velocity_pub_, pc_with_velocity, left_flow->header);

    if (static_flow_pub_.getNumSubscribers() > 0)
      static_flow_pub_.publish(left_static_flow.toImageMsg());
  }
}

void SceneFlowConstructor::constructVelocityPC
(
  const pcl::PointCloud<pcl::PointXYZ> &pc_now,
  const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed,
  cv_bridge::CvImage &left_flow,
  cv::Mat &left_static_flow,
  DisparityImageProcessor &disparity_now,
  DisparityImageProcessor &disparity_previous,
  pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc
)
{
  initializeVelocityPC(velocity_pc);

  ros::Time stamp_now = disparity_now._disparity_msg.header.stamp;
  ros::Time stamp_previous = disparity_previous._disparity_msg.header.stamp;
  ros::Duration time_between_frames = stamp_now - stamp_previous;

  cv::Point2i left_now;
  for (left_now.y = 0; left_now.y < image_height_; left_now.y++)
  {
    for (left_now.x = 0; left_now.x < image_width_; left_now.x++)
    {
      pcl::PointXYZVelocity &point_with_velocity = velocity_pc.at(left_now.x, left_now.y);
      pcl::PointXYZ point3d_now = pc_now.at(left_now.x, left_now.y);

      if (!isValid(point3d_now))
        continue;

      point_with_velocity.x = point3d_now.x;
      point_with_velocity.y = point3d_now.y;
      point_with_velocity.z = point3d_now.z;

      cv::Point2i left_previous, right_now, right_previous;

      if (!getMatchPoints(left_now, left_previous, right_now, right_previous, left_flow, disparity_now, disparity_previous))
        continue;

      pcl::PointXYZ point3d_previous;
      point3d_previous = pc_previous_transformed.at(left_previous.x, left_previous.y);
      if (!isValid(point3d_previous))
        continue;

      const cv::Vec2f &flow = left_flow.image.at<cv::Vec2f>(left_now);;
      cv::Vec2f static_flow = left_static_flow.at<cv::Vec2f>(left_now);
      if (std::isnan(static_flow[0]))
        continue;

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

void SceneFlowConstructor::estimateCameraMotion(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info)
{
  if (!visual_odometer_)
    initializeOdometer(*left_camera_info, *right_camera_info);

  // convert images
  cv_bridge::CvImageConstPtr cv_left = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImageConstPtr cv_right = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::MONO8);

  // assertion for input images
  ROS_ASSERT(cv_left->image.step[0] == cv_right->image.step[0]);
  ROS_ASSERT(cv_left->image.rows == cv_right->image.rows);
  ROS_ASSERT(cv_left->image.cols == cv_left->image.cols);

  // estimate camera motion
  int32_t dims[] = {cv_left->image.cols, cv_left->image.rows, static_cast<int32_t>(cv_left->image.step[0])};
  bool success = visual_odometer_->process(cv_left->image.data, cv_right->image.data, dims);

  if (success)
  {
    // Left camera motion from previous frame to current frame
    Matrix camera_motion = visual_odometer_->getMotion();

    tf2::Matrix3x3 camera_rotation
    (
      camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
      camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
      camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]
    );
    tf2::Vector3 camera_translation(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
    tf2::Transform tf2_camera_motion(camera_rotation, camera_translation);

    integrateAndBroadcastTF(tf2_camera_motion.inverse(), left_image->header.stamp);

    transform_prev2now_.reset(new geometry_msgs::Transform());
    *transform_prev2now_ = tf2::toMsg(tf2_camera_motion);
  }
  else
  {
    transform_prev2now_.reset();
    ROS_ERROR_STREAM("Visual odometry is failed\nInput timestamp: " << left_image->header.stamp);
  }
}

void SceneFlowConstructor::estimateDisparity
(
  const sensor_msgs::ImageConstPtr& left_image, 
  const sensor_msgs::ImageConstPtr& right_image, 
  const sensor_msgs::CameraInfoConstPtr& left_camera_info, 
  const sensor_msgs::CameraInfoConstPtr& right_camera_info
)
{
  stereo_msgs::DisparityImage disparity;
  bool success = sgm_gpu_->computeDisparity(*left_image, *right_image, 
    *left_camera_info, *right_camera_info, disparity);

  if (success)
    disparity_now_.reset(new DisparityImageProcessor(disparity, *left_camera_info));
  else
  {
    disparity_now_.reset();
    ROS_ERROR_STREAM("Disparity estimation is failed\nInput timestamp: " << left_image->header.stamp);
  }
}

void SceneFlowConstructor::estimateOpticalFlow(const sensor_msgs::ImageConstPtr& left_image)
{
  left_flow_.reset(new cv_bridge::CvImage(left_image->header, sensor_msgs::image_encodings::TYPE_32FC2));
  bool success = pwc_net_.estimateOpticalFlow(*previous_left_image_, *left_image, left_flow_->image);

  if (!success)
  {
    left_flow_.reset();
    ROS_ERROR_STREAM("Optical flow estimation is failed\nInput timestamp: " 
      << previous_left_image_->header.stamp << " and " << left_image->header.stamp);
  }
}
  

void SceneFlowConstructor::initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc)
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

void SceneFlowConstructor::initializeOdometer(const sensor_msgs::CameraInfo& l_info_msg, const sensor_msgs::CameraInfo& r_info_msg)
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
  ROS_DEBUG_STREAM("Initialized libviso2 stereo odometry with the following parameters:\n" << visual_odometer_params_);
}

void SceneFlowConstructor::integrateAndBroadcastTF(const tf2::Transform& delta_transform, const ros::Time& timestamp)
{
  integrated_pose_ *= delta_transform;

  // transform integrated pose to base frame
  std::string error_msg;
  tf2::Stamped<tf2::Transform> base_to_sensor;
  if (tf_buffer_.canTransform(base_link_frame_id_, camera_frame_id_, timestamp, &error_msg))
  {
    geometry_msgs::TransformStamped base_to_sensor_msg;
    base_to_sensor_msg = tf_buffer_.lookupTransform(base_link_frame_id_, camera_frame_id_, timestamp);
    tf2::fromMsg(base_to_sensor_msg, base_to_sensor);
  }
  else
  {
    ROS_ERROR_THROTTLE(10.0, 
      "The tf from '%s' to '%s' does not seem to be available, will assume it as identity!",
      base_link_frame_id_.c_str(),
      camera_frame_id_.c_str()
    );
    ROS_ERROR_THROTTLE(10.0, "Transform error: %s", error_msg.c_str());
    base_to_sensor.setIdentity();
  }

  tf2::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();

  geometry_msgs::TransformStamped base_transform_msg = tf2::toMsg(tf2::Stamped<tf2::Transform>(base_transform, timestamp, odom_frame_id_));
  base_transform_msg.child_frame_id = base_link_frame_id_;
  tf_broadcaster_.sendTransform(base_transform_msg);
}

template <typename PointT> void SceneFlowConstructor::publishPointcloud
(
  const ros::Publisher &publisher,
  const pcl::PointCloud<PointT> &pointcloud,
  const std_msgs::Header &header
)
{
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pointcloud, pointcloud_msg);
  pointcloud_msg.header = header;
  publisher.publish(pointcloud_msg);
}

void SceneFlowConstructor::stereoCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info)
{
  ros::WallTime start_process = ros::WallTime::now();

  if (!left_cam_model_)
  {
    left_cam_model_.reset(new image_geometry::PinholeCameraModel());
    left_cam_model_->fromCameraInfo(left_camera_info);
    camera_frame_id_ = left_image->header.frame_id;
    image_width_ = left_image->width;
    image_height_ = left_image->height;
  }

  ROS_DEBUG("Get disparity, optical flow and camera motion by calling external services on separate threads");
  std::thread disparity_thread(&SceneFlowConstructor::estimateDisparity, this, left_image, right_image, left_camera_info, right_camera_info);
  std::thread cammotion_thread(&SceneFlowConstructor::estimateCameraMotion, this, left_image, right_image, left_camera_info, right_camera_info);
  if (previous_left_image_)
  {
    std::thread optflow_thread(&SceneFlowConstructor::estimateOpticalFlow, this, left_image);
    optflow_thread.join();
  }
  disparity_thread.join();
  cammotion_thread.join();
  ROS_DEBUG("Threads for disparity, optical flow and camera motion are finished");

  if (construct_thread_.joinable())
    construct_thread_.join();

  construct_thread_ = std::thread(&SceneFlowConstructor::construct, this, disparity_now_, disparity_previous_, left_flow_, transform_prev2now_);

  ros::WallDuration process_time = ros::WallTime::now() - start_process;
  ROS_INFO("process time: %f", process_time.toSec());

  previous_left_image_ = left_image;
  disparity_previous_ = disparity_now_;
}

void SceneFlowConstructor::reconfigureCB(scene_flow_constructor::SceneFlowConstructorConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: dynamic_flow_diff = %d, max_color_velocity = %f", config.dynamic_flow_diff, config.max_color_velocity);

  dynamic_flow_diff_  = config.dynamic_flow_diff;
  max_color_velocity_ = config.max_color_velocity;
}

void SceneFlowConstructor::transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &previous_to_now)
{
  Eigen::Isometry3d eigen_prev2now = tf2::transformToEigen(previous_to_now);

  pc_previous_transformed = pcl::PointCloud<pcl::PointXYZ>(image_width_, image_height_);
  for (int u = 0; u < image_width_; u++)
  {
    for (int v = 0; v < image_height_; v++)
    {
      const pcl::PointXYZ &point = pc_previous.at(u, v);
      if (std::isnan(point.x))
      {
        pc_previous_transformed.at(u, v) = pc_previous.at(u, v);
        continue;
      }

      Eigen::Vector3d eigen_transformed = eigen_prev2now * Eigen::Vector3d(point.x, point.y, point.z);
      pc_previous_transformed.at(u, v) = pcl::PointXYZ(eigen_transformed.x(), eigen_transformed.y(), eigen_transformed.z());
    }
  }
}

void SceneFlowConstructor::publishDepthImage(ros::Publisher& depth_pub, cv::Mat& depth_image, ros::Time timestamp)
{
  std_msgs::Header header;
  header.frame_id = camera_frame_id_;
  header.stamp = timestamp;
  cv_bridge::CvImage depth_bridge(header, "32FC1", depth_image);
  depth_pub.publish(depth_bridge.toImageMsg());
}

} // namespace scene_flow_constructor
