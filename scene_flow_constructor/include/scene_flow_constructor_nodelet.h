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

#include <thread>

namespace scene_flow_constructor{

class SceneFlowConstructorNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();
private:
  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  /**
    * \brief Thread to execute construct()
    */
  std::thread construct_thread_;
  
  /**
   * \brief Publisher for optical flow of left image
   */
  ros::Publisher optflow_pub_;
  /**
   * \brief Publisher for disparity at now frame
   */
  ros::Publisher depth_previous_pub_;
  ros::Publisher depth_now_pub_;
  ros::Publisher colored_pc_pub_;
  ros::Publisher colored_pc_relative_pub_;
  ros::Publisher pc_with_velocity_pub_;
  ros::Publisher pc_with_relative_velocity_pub_;
  ros::Publisher static_flow_pub_;
  image_transport::Publisher velocity_image_pub_;
  image_transport::Publisher flow_residual_pub_;

  image_transport::Publisher left_previous_pub_;
  image_transport::Publisher left_now_pub_;
  image_transport::Publisher right_now_pub_;
  image_transport::Publisher right_previous_pub_;

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

  ros::ServiceClient pause_service_client_;

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

  bool bag_auto_pause_;
  
  /**
   * \brief Difference[pixel] between optical flow and calculated static optical flow treated as dynamic pixel
   */
  int dynamic_flow_diff_;

  /**
   * \brief Parameter used by visualization of velocity pc on image plane
   * When velocity of point is faster than this parameter, maximum color intensity is assigned at corresponded pixel
   */
  double max_color_velocity_;

  sensor_msgs::ImageConstPtr previous_left_image_;
  sensor_msgs::ImageConstPtr previous_right_image_;
  std::shared_ptr<DisparityImageProcessor> disparity_previous_;
  std::shared_ptr<DisparityImageProcessor> disparity_now_;
  optical_flow_msgs::DenseOpticalFlowPtr left_flow_;
  geometry_msgs::TransformPtr transform_prev2now_;

  std::string camera_frame_id_;

  int image_width_;
  int image_height_;

  std::shared_ptr<image_geometry::PinholeCameraModel> left_cam_model_;

  /**
   * \brief Calculate optical flow of left frame with static assumption
   */
  void calculateStaticOpticalFlow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, cv::Mat &left_static_flow);

  /**
   * \brief construct various data from disparity prev/now, left optical flow and camera movement
   */
  void construct(std::shared_ptr<DisparityImageProcessor> disparity_now, std::shared_ptr<DisparityImageProcessor> disparity_previous, optical_flow_msgs::DenseOpticalFlowPtr left_flow, geometry_msgs::TransformPtr transform_prev2now);

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
   */
  void constructVelocityPC(const pcl::PointCloud<pcl::PointXYZ> &pc_now, const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, optical_flow_msgs::DenseOpticalFlow &left_flow, cv::Mat &left_static_flow, DisparityImageProcessor &disparity_now, DisparityImageProcessor &disparity_previous, const ros::Duration &time_between_frames, pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);

  /**
   * \brief Calculate relative velocity from camera of each point and construct pointcloud.
   */
  void constructVelocityPCRelative(const pcl::PointCloud<pcl::PointXYZ> &pc_now, const pcl::PointCloud<pcl::PointXYZ> &pc_previous, const ros::Duration &time_between_frames, optical_flow_msgs::DenseOpticalFlow &left_flow, DisparityImageProcessor &disparity_now, DisparityImageProcessor &disparity_previous, pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);

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
  inline bool getMatchPoints(const cv::Point2i &left_now, cv::Point2i &left_previous, cv::Point2i &right_now, cv::Point2i &right_previous, optical_flow_msgs::DenseOpticalFlow &left_flow, DisparityImageProcessor &disparity_now, DisparityImageProcessor &disparity_previous)
  {
    if (!getPreviousPoint(left_now, left_previous, left_flow))
      return false;

    if (!getRightPoint(left_now, right_now, disparity_now))
      return false;

    if (!getRightPoint(left_previous, right_previous, disparity_previous))
      return false;

    return true;
  }
  inline bool getPreviousPoint(const cv::Point2i &now, cv::Point2i &previous, optical_flow_msgs::DenseOpticalFlow &flow)
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
  inline bool getRightPoint(const cv::Point2i &left, cv::Point2i &right, DisparityImageProcessor &disparity_processor)
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
  void initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);
  inline bool isValid(const pcl::PointXYZ &point)
  {
    if (std::isnan(point.x))
      return false;

    if (std::isinf(point.x))
      return false;

    return true;
  }

  /**
   * \brief Publish residual image between estimated optical flow and static optical flow
   * 
   * Pixel value of the image is Euclidean norm between two flows.
   */
  void publishFlowResidual(cv::Mat& left_static_flow, optical_flow_msgs::DenseOpticalFlowPtr& left_flow, const ros::Time &time_now);
  /**
   * \brief Publish static optical flow of left frame
   * 
   * The flow is synthesis optical flow calculated by camera transform and depth image, static scene assumption
   */
  void publishStaticOpticalFlow(cv::Mat& left_static_flow, const ros::Time &time_now, const ros::Time &time_previous);
  /**
   * \brief Publish image which visualize velocity pc by RGB color
   *
   * \param velocity_image Already constructed velocity image
   */
  void publishVelocityImage(const cv::Mat &velocity_image, const ros::Time time_now);

  void publishDepthImage(ros::Publisher& depth_pub, cv::Mat& depth_image, ros::Time timestamp);

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
