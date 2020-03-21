#ifndef SCENE_FLOW_CONSTRUCTOR__SCENE_FLOW_CONSTRUCTOR_H_
#define SCENE_FLOW_CONSTRUCTOR__SCENE_FLOW_CONSTRUCTOR_H_

#include <cv_bridge/cv_bridge.h>
#include <disparity_image_proc/disparity_image_processor.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pwc_net/pwc_net.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sgm_gpu/sgm_gpu.h>
#include <stereo_msgs/DisparityImage.h>
#include <scene_flow_constructor/pcl_point_xyz_velocity.h>
#include <scene_flow_constructor/SceneFlowConstructorConfig.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <viso_stereo.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>

namespace scene_flow_constructor{

class SceneFlowConstructor {
public:
  SceneFlowConstructor();
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
  ros::Publisher depth_pub_;
  ros::Publisher pc_with_velocity_pub_;
  ros::Publisher static_flow_pub_;

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

  sensor_msgs::ImageConstPtr previous_left_image_;
  std::shared_ptr<DisparityImageProcessor> disparity_previous_;
  std::shared_ptr<DisparityImageProcessor> disparity_now_;
  std::shared_ptr<cv_bridge::CvImage> left_flow_;
  geometry_msgs::TransformPtr transform_prev2now_;

  std::string camera_frame_id_;

  int image_width_;
  int image_height_;

  std::shared_ptr<image_geometry::PinholeCameraModel> left_cam_model_;

  /**
   * \brief Stereo visual odometry class from libviso2
   */
  std::shared_ptr<VisualOdometryStereo> visual_odometer_;
  /**
   * \brief Parameters used to initialize visual_odometer_
   */
  VisualOdometryStereo::parameters visual_odometer_params_;

  // Frame IDs for visual odometry
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  /**
   * \brief Camera pose which camera motions of each frame are cumulated
   */
  tf2::Transform integrated_pose_;

  std::shared_ptr<sgm_gpu::SgmGpu> sgm_gpu_;

  pwc_net::PwcNet pwc_net_;

  /**
   * \brief Calculate optical flow of left frame with static assumption
   */
  void calculateStaticOpticalFlow(const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed, cv::Mat &left_static_flow);

  /**
   * \brief construct various data from disparity prev/now, left optical flow and camera movement
   */
  void construct
  (
    std::shared_ptr<DisparityImageProcessor> disparity_now,
    std::shared_ptr<DisparityImageProcessor> disparity_previous,
    std::shared_ptr<cv_bridge::CvImage> left_flow,
    geometry_msgs::TransformPtr transform_prev2now
  );

  /**
   * \brief Calculate velocity of each point and construct pointcloud.
   */
  void constructVelocityPC
  (
    const pcl::PointCloud<pcl::PointXYZ> &pc_now,
    const pcl::PointCloud<pcl::PointXYZ> &pc_previous_transformed,
    cv_bridge::CvImage &left_flow,
    cv::Mat &left_static_flow,
    DisparityImageProcessor &disparity_now,
    DisparityImageProcessor &disparity_previous,
    pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc
  );

  /**
   * \brief Estimate left camera motion by LIBVISO2
   */
  void estimateCameraMotion(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);
  /**
   * \brief Estimate disparity by SGM
   */
  void estimateDisparity(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);
  /**
   * \brief Estimate optical flow by PWC-Net
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
  inline bool getMatchPoints
  (
    const cv::Point2i &left_now,
    cv::Point2i &left_previous,
    cv::Point2i &right_now,
    cv::Point2i &right_previous,
    cv_bridge::CvImage &left_flow,
    DisparityImageProcessor &disparity_now, 
    DisparityImageProcessor &disparity_previous
  )
  {
    if (!getPreviousPoint(left_now, left_previous, left_flow))
      return false;

    if (!getRightPoint(left_now, right_now, disparity_now))
      return false;

    if (!getRightPoint(left_previous, right_previous, disparity_previous))
      return false;

    return true;
  }
  inline bool getPreviousPoint
  (
    const cv::Point2i &now,
    cv::Point2i &previous,
    cv_bridge::CvImage &flow
  )
  {
    if (now.x < 0 || now.x >= image_width_ || now.y < 0 || now.y >= image_height_)
      return false;

    const cv::Vec2f &flow_at_point = flow.image.at<cv::Vec2f>(now);

    if (std::isnan(flow_at_point[0]) || std::isnan(flow_at_point[1]))
      return false;

    previous.x = std::round(now.x - flow_at_point[0]);
    previous.y = std::round(now.y - flow_at_point[1]);

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

  void initializeOdometer(const sensor_msgs::CameraInfo& l_info_msg, const sensor_msgs::CameraInfo& r_info_msg);

  /**
   * \brief Resize velocity pointcloud and fill each point by default value
   *
   * \param velocity_pc Target velocity pointcloud
   */
  void initializeVelocityPC(pcl::PointCloud<pcl::PointXYZVelocity> &velocity_pc);

  void integrateAndBroadcastTF(const tf2::Transform& delta_transform, const ros::Time& timestamp);

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
  void publishFlowResidual
    (cv_bridge::CvImage& left_static_flow, cv_bridge::CvImage& left_flow);
  /**
   * \brief Publish image which visualize velocity pc by RGB color
   *
   * \param velocity_image Already constructed velocity image
   */
  void publishVelocityImage(const cv::Mat &velocity_image, const ros::Time time_now);

  void publishDepthImage(ros::Publisher& depth_pub, cv::Mat& depth_image, ros::Time timestamp);

  template <typename PointT> void publishPointcloud
  (
    const ros::Publisher &publisher,
    const pcl::PointCloud<PointT> &pointcloud,
    const std_msgs::Header &header
  );

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

#endif // SCENE_FLOW_CONSTRUCTOR__SCENE_FLOW_CONSTRUCTOR_H_
