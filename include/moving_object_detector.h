#ifndef __HEADER_MOVING_OBJECT_DETECTOR__
#define __HEADER_MOVING_OBJECT_DETECTOR__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2/LinearMath/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>

class MovingObjectDetector {
public:
  MovingObjectDetector();
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher flow3d_pub_;
  ros::Publisher cluster_pub_;
  ros::Publisher debug_pub_;
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub_;
  message_filters::Subscriber<opencv_apps::FlowArrayStamped> optical_flow_sub_;
  
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_image_info_sub_;
  
  std::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, opencv_apps::FlowArrayStamped, sensor_msgs::Image, sensor_msgs::CameraInfo>> time_sync_;
  
  double moving_flow_length_;
  double flow_length_diff_;
  double flow_start_diff_;
  double flow_radian_diff_;
  double flow_axis_max_;
  int cluster_element_num_;
  
  image_geometry::PinholeCameraModel camera_model_;
  sensor_msgs::Image depth_image_previous_;
  ros::Time time_stamp_previous_;
  
  bool first_run_;
  
  void dataCB(const geometry_msgs::TransformStampedConstPtr&, const opencv_apps::FlowArrayStampedConstPtr&, const sensor_msgs::ImageConstPtr&, const sensor_msgs::CameraInfoConstPtr&);
  
  template<typename T>
  bool getPoint3D_internal(int, int, const sensor_msgs::Image&, tf2::Vector3&);
  bool getPoint3D(int, int, const sensor_msgs::Image&, tf2::Vector3&);
  
  // 同期した各入力データをsubscribeし，optical flowノード，VISO2ノード，rectifyノードにタイミング良くpublishするためのクラス
  class InputSynchronizer {
  private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    
    image_transport::CameraPublisher depth_image_pub_;
    image_transport::CameraPublisher left_rect_image_pub_;
    image_transport::CameraPublisher right_rect_image_pub_;
    
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_; // depth imageのフォーマットはcompress transportに対応していないため，image_transportは使用しない
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
    image_transport::SubscriberFilter left_rect_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_rect_info_sub_;
    image_transport::SubscriberFilter right_rect_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_rect_info_sub_;
    
    typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> DataTimeSynchronizer;
    std::shared_ptr<DataTimeSynchronizer> time_sync_;
    
    sensor_msgs::Image depth_image_;
    sensor_msgs::CameraInfo depth_image_info_;
    sensor_msgs::Image left_rect_image_;
    sensor_msgs::CameraInfo left_rect_info_;
    sensor_msgs::Image right_rect_image_;
    sensor_msgs::CameraInfo right_rect_info_;
    
    ros::Time last_publish_time_;
    
    void dataCallBack(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info);
    
  public:
    InputSynchronizer(ros::NodeHandle& node_handle);
    void publish();
  };
  
  std::shared_ptr<InputSynchronizer> input_synchronizer_;
};

#endif
