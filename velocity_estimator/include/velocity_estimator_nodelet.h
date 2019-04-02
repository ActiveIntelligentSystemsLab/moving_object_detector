#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <disparity_image_proc/disparity_image_processor.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <dense_flow_msg/DenseFlow.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <velocity_estimator/pcl_point_xyz_velocity.h>
#include <velocity_estimator/VelocityEstimatorConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <list>

namespace velocity_estimator{

class VelocityEstimatorNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::shared_ptr<image_transport::ImageTransport> image_transport;
  
  ros::Publisher pc_with_velocity_pub_;
  ros::Publisher static_flow_pub_;
  image_transport::Publisher velocity_image_pub_;
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub_;
  message_filters::Subscriber<dense_flow_msg::DenseFlow> optical_flow_left_sub_;
  message_filters::Subscriber<dense_flow_msg::DenseFlow> optical_flow_right_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;

  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, dense_flow_msg::DenseFlow, dense_flow_msg::DenseFlow, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>> time_sync_;
  
  dynamic_reconfigure::Server<velocity_estimator::VelocityEstimatorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<velocity_estimator::VelocityEstimatorConfig>::CallbackType reconfigure_func_;
  
  double matching_tolerance_;

  /**
   * \brief Parameter used by visualization of velocity pc on image plane
   * When velocity of point is faster than this parameter, maximum color intensity is assigned at corresponded pixel
   */
  double max_color_velocity_;

  cv::Mat left_flow_, right_flow_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_now_, pc_previous_;

  /**
   * \brief Pointcloud of previous frame transformed by camera motion matrix from previous to now
   */
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_previous_transformed_;
  std::shared_ptr<DisparityImageProcessor> disparity_now_, disparity_previous_;
  geometry_msgs::TransformStamped transform_now_to_previous_;
  ros::Time time_stamp_previous_;

  image_geometry::PinholeCameraModel left_cam_model_;

  /**
   * \brief Calculate optical flow with static assumption
   * \param flow calculated flow by this function
   */
  void calculateStaticOpticalFlow(cv::Mat *flow);

  /**
   * \brief Construct velocity image which visualize velocity_pc as RGB color image
   *
   * \param velocity_pc Input pointcloud whose points have 3D position and velocity
   * \param velocity_image Output image visualizes velocity by RGB color
   */
  void constructVelocityImage(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, cv::Mat &velocity_image);

  /**
   * \brief Calculate velocity of each point and construct pointcloud.
   *
   * \param velocity_pc Pointcloud whose points have 3D position and velocity
   */
  void constructVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);
  void dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const dense_flow_msg::DenseFlowConstPtr& optical_flow_left, const dense_flow_msg::DenseFlowConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image);

  /**
   * \brief Get points in 3 images (left previous, right now  and right previous frame) which match to a point in left now image
   *
   * \param left_now A point in left now image.
   * \param left_previous A point in left previous image matched to left_now.
   * \param right_now A point in right now image matched to left_now.
   * \param right_previous A point in right previous image matched to left_now.
   *
   * \return Return false if there aren't three match points or matching error is bigger than matching_tolerance_.
   */
  bool getMatchPoints(const cv::Point2i &left_now, cv::Point2i &left_previous, cv::Point2i &right_now, cv::Point2i &right_previous);
  bool getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const cv::Mat &flow);
  bool getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor);
  void initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);
  bool isValid(const pcl::PointXYZ &point);

  /**
   * \brief Publish static optical flow
   *
   * \param Calculated static optical flow
   */
  void publishStaticOpticalFlow(const cv::Mat &static_flow);

  /**
   * \brief Publish image which visualize velocity pc by RGB color
   *
   * \param velocity_image Already constructed velocity image
   */
  void publishVelocityImage(const cv::Mat &velocity_image);
  template <typename PointT> void publishPointcloud(const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp);
  void reconfigureCB(velocity_estimator::VelocityEstimatorConfig& config, uint32_t level);

  /**
   * \brief Transform pointcloud of previous frame to now frame
   *
   * \param pc_previous Pointcloud of previous frame
   * \param pc_previous_transformed Transformed pointcloud of previous frame
   * \param now_to_previous Transformation from now frame to previous frame
   */
  void transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &now_to_previous);
};

} // namespace velocity_estimator

#endif
