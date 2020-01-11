#ifndef SCENE_FLOW_CONSTRUCTOR_H
#define SCENE_FLOW_CONSTRUCTOR_H

#include <disparity_image_proc/disparity_image_processor.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <optical_flow_msgs/DenseOpticalFlow.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <scene_flow_constructor/pcl_point_xyz_velocity.h>
#include <scene_flow_constructor/SceneFlowConstructorConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <list>

namespace scene_flow_constructor{

class SceneFlowConstructorNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  
  /**
   * \brief Publisher for optical flow of left image
   */
  ros::Publisher optflow_pub_;
  /**
   * \brief Publisher for disparity at now frame
   */
  ros::Publisher disparity_pub_;
  ros::Publisher colored_pc_pub_;
  ros::Publisher pc_with_velocity_pub_;
  ros::Publisher static_flow_pub_;
  image_transport::Publisher velocity_image_pub_;
  /**
   * \brief Publish residual between estimated optical flow and synthesis optical flow with static scene assumption
   */
  image_transport::Publisher flow_residual_pub_;

  /**
   * \brief Client for EstimateMotionFromStereo service
   */
  ros::ServiceClient motion_service_client_;
  /**
   * \brief Client for EstimateDisparity service
   */
  ros::ServiceClient disparity_service_client_;
  /**
   * \brief Client for CalculateDenseOpticalFlow service
   */
  ros::ServiceClient optflow_service_client_;

  // Stereo image and camera info subscribers
  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_caminfo_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_caminfo_sub_;

  using StereoSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>;
  /**
   * \brief Time synchronizer of stereo image and camera info 
   */
  std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;
  
  using ReconfigureServer = dynamic_reconfigure::Server<scene_flow_constructor::SceneFlowConstructorConfig>;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;
  ReconfigureServer::CallbackType reconfigure_func_;
  
  /**
   * \brief Difference[pixel] between optical flow and calculated static optical flow treated as dynamic pixel
   */
  int dynamic_flow_diff_;

  /**
   * \brief Parameter used by visualization of velocity pc on image plane
   * When velocity of point is faster than this parameter, maximum color intensity is assigned at corresponded pixel
   */
  double max_color_velocity_;

  optical_flow_msgs::DenseOpticalFlowPtr left_flow_;

  // Stereo image of previous frame
  sensor_msgs::ImageConstPtr previous_left_image_;
  sensor_msgs::ImageConstPtr previous_right_image_;

  /**
   * \brief Optical flow of left frame calculated by assuming static scene from camera motion and 3D reconstructed pointcloud
   */
  cv::Mat left_static_flow_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_now_, pc_previous_;

  /**
   * \brief Pointcloud of previous frame transformed by camera motion matrix from previous to now
   */
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_previous_transformed_;
  std::shared_ptr<DisparityImageProcessor> disparity_now_, disparity_previous_;
  /**
   * \brief Left camera motion from previous frame to current frame
   *
   * Estimated by visual odometry
   */
  geometry_msgs::TransformPtr transform_prev2now_;
  //geometry_msgs::TransformStamped transform_now_to_previous_;

  ros::Time time_stamp_now_;
  ros::Time time_stamp_previous_;

  std::string camera_frame_id_;

  /**
   * \brief Width of optical flow, disparity image and also pointcloud
   */
  int image_width_;
  /**
   * \brief Height of optical flow, disparity image and also pointcloud
   */
  int image_height_;

  image_geometry::PinholeCameraModel left_cam_model_;

  /**
   * \brief Calculate optical flow of left frame with static assumption
   */
  void calculateStaticOpticalFlow();

  /**
   * \brief construct RGB colored pointcloud which visualize velocity
   *
   * \param velocity_pc input pointcloud whose points have 3d position and velocity
   * \param velocity_image output pointcloud visualizes velocity by rgb color
   */
  void constructVelocityColoredPC(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, pcl::PointCloud<pcl::PointXYZRGB> &colored_pc);

  /**
   * \brief construct velocity image which visualize velocity_pc as rgb color image
   *
   * \param velocity_pc input pointcloud whose points have 3d position and velocity
   * \param velocity_image output image visualizes velocity by rgb color
   */
  void constructVelocityImage(const pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc, cv::Mat &velocity_image);

  /**
   * \brief Calculate velocity of each point and construct pointcloud.
   *
   * \param velocity_pc Pointcloud whose points have 3D position and velocity
   */
  void constructVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);

  /**
   * \brief Estimate left camera motion by calling external service
   */
  void estimateCameraMotion(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);
  /**
   * \brief Estimate disparity by calling external service
   */
  void estimateDisparity(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);
  /**
   * \brief Estimate optical flow by calling external service
   */
  void estimateOpticalFlow(const sensor_msgs::ImageConstPtr& left_image);

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
  bool getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, const optical_flow_msgs::DenseOpticalFlow &flow);
  bool getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor);

  /**
   * \brief Resize velocity pointcloud and fill each point by default value
   *
   * \param velocity_pc Target velocity pointcloud
   */
  void initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);
  bool isValid(const pcl::PointXYZ &point);

  /**
   * \brief Publish residual image between estimated optical flow and static optical flow
   * 
   * Pixel value of the image is Euclidean norm between two flows.
   */
  void publishFlowResidual();
  /**
   * \brief Publish static optical flow of left frame
   * 
   * The flow is synthesis optical flow calculated by camera transform and depth image, static scene assumption
   */
  void publishStaticOpticalFlow();
  /**
   * \brief Publish image which visualize velocity pc by RGB color
   *
   * \param velocity_image Already constructed velocity image
   */
  void publishVelocityImage(const cv::Mat &velocity_image);

  template <typename PointT> void publishPointcloud(const ros::Publisher &publisher, const pcl::PointCloud<PointT> &pointcloud, const std::string &frame_id, const ros::Time &stamp);
  void reconfigureCB(scene_flow_constructor::SceneFlowConstructorConfig& config, uint32_t level);

  /**
   * \brief Callback function of stereo_synchronizer_
   */
  void stereoCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);

  /**
   * \brief Transform pointcloud of previous frame to now frame
   *
   * \param pc_previous Pointcloud of previous frame
   * \param pc_previous_transformed Transformed pointcloud of previous frame
   * \param previous_to_now Transform from now frame to previous frame
   */
  void transformPCPreviousToNow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous, pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, const geometry_msgs::Transform &previous_to_now);
};

} // namespace scene_flow_constructor

#endif
