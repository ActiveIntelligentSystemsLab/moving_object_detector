#ifndef __HEADER_MOVING_OBJECT_DETECTOR__
#define __HEADER_MOVING_OBJECT_DETECTOR__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv_apps/FlowMap.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <stereo_msgs/DisparityImage.h>
#include <dynamic_reconfigure/server.h>
#include <moving_object_detector/MovingObjectDetectorConfig.h>

class MovingObjectDetector {
public:
  MovingObjectDetector();
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher flow3d_pub_;
  ros::Publisher cluster_pub_;
  ros::Publisher removed_by_matching_pub_;
  ros::Publisher removed_by_confidence_pub_;
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub_;
  message_filters::Subscriber<sensor_msgs::Image> optical_flow_left_sub_;
  message_filters::Subscriber<sensor_msgs::Image> optical_flow_right_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> confidence_map_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_image_info_sub_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;

  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, stereo_msgs::DisparityImage>> time_sync_;
  
  dynamic_reconfigure::Server<moving_object_detector::MovingObjectDetectorConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_detector::MovingObjectDetectorConfig>::CallbackType reconfigure_func_;
  
  int downsample_scale_;
  int confidence_limit_;
  double moving_flow_length_;
  double flow_length_diff_;
  double flow_start_diff_;
  double flow_radian_diff_;
  double flow_axis_max_;
  double matching_tolerance_;
  int cluster_element_num_;
  
  image_geometry::PinholeCameraModel camera_model_;
  
  sensor_msgs::Image depth_image_previous_;
  cv::Mat confidence_previous_;
  cv::Mat disparity_map_previous_;
  ros::Time time_stamp_previous_;
  
  bool first_run_;
  
  void reconfigureCB(moving_object_detector::MovingObjectDetectorConfig& config, uint32_t level);
  void dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const sensor_msgs::ImageConstPtr& optical_flow_left, const sensor_msgs::ImageConstPtr& optical_flow_right, const sensor_msgs::ImageConstPtr& depth_image_now, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& confidence_map, const stereo_msgs::DisparityImageConstPtr& disparity_image);
  
  // 同期した各入力データをsubscribeし，optical flowノード，VISO2ノード，rectifyノードにタイミング良くpublishするためのクラス
  class InputSynchronizer {
  private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    
    image_transport::CameraPublisher depth_image_pub_;
    ros::Publisher confidence_map_pub_;    
    image_transport::CameraPublisher left_rect_image_pub_;
    image_transport::CameraPublisher right_rect_image_pub_;
    ros::Publisher disparity_image_pub_;
    
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_; // depth imageのフォーマットはcompress transportに対応していないため，image_transportは使用しない
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image> confidence_map_sub_;
    image_transport::SubscriberFilter left_rect_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_rect_info_sub_;
    image_transport::SubscriberFilter right_rect_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_rect_info_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_image_sub_;
    
    typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage> DataTimeSynchronizer;
    std::shared_ptr<DataTimeSynchronizer> time_sync_;
    
    sensor_msgs::Image depth_image_;
    sensor_msgs::CameraInfo depth_image_info_;
    sensor_msgs::Image confidence_map_;
    sensor_msgs::Image left_rect_image_;
    sensor_msgs::CameraInfo left_rect_info_;
    sensor_msgs::Image right_rect_image_;
    sensor_msgs::CameraInfo right_rect_info_;
    stereo_msgs::DisparityImage disparity_image_;
    
    ros::Time last_publish_time_;
    
    void dataCallBack(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& confidence_map, const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info, const stereo_msgs::DisparityImageConstPtr& disparity_image);
    
  public:
    InputSynchronizer(MovingObjectDetector& outer_instance);
    void publish();
  };
  
  std::shared_ptr<InputSynchronizer> input_synchronizer_;
};

#endif
