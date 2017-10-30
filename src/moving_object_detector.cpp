#include "moving_object_detector.h"
#include "flow_3d.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/camera_common.h>
#include <algorithm>
#include <limits>
#include <vector>

// depth_image_procパッケージより
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) { return depth != 0; }
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
  static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }
  
  static inline void initializeBuffer(std::vector<uint8_t>& buffer)
  {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};

MovingObjectDetector::MovingObjectDetector() {  
  first_run_ = true;
  
  ros::param::param("~moving_flow_length", moving_flow_length_, 0.05);
  ros::param::param("~flow_length_diff", flow_length_diff_, 0.05);
  ros::param::param("~flow_start_diff", flow_start_diff_, 0.10);
  ros::param::param("~flow_radian_diff", flow_radian_diff_, 0.17);
  
  point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("clustered_point_cloud", 10);
  
  std::string depth_image_topic = node_handle_.resolveName("depth_image_rectified"); // image_transport::SubscriberFilter は何故か名前解決してくれないので
  std::string depth_image_info_topic = image_transport::getCameraInfoTopic(depth_image_topic);
  
  camera_transform_sub_.subscribe(node_handle_, "camera_transform", 1);
  optical_flow_sub_.subscribe(node_handle_, "optical_flow", 1); // optical flowはrectified imageで計算すること
  depth_image_sub_.subscribe(node_handle_, depth_image_topic, 1);
  depth_image_info_sub_.subscribe(node_handle_, depth_image_info_topic, 1);
  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, opencv_apps::FlowArrayStamped, sensor_msgs::Image, sensor_msgs::CameraInfo>>(camera_transform_sub_, optical_flow_sub_, depth_image_sub_, depth_image_info_sub_, 1);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::dataCB, this, _1, _2, _3, _4));
  
  input_synchronizer_ = std::make_shared<InputSynchronizer>(node_handle_);
}

void MovingObjectDetector::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const opencv_apps::FlowArrayStampedConstPtr& optical_flow, const sensor_msgs::ImageConstPtr& depth_image_now, const sensor_msgs::CameraInfoConstPtr& depth_image_info)
{
  camera_model_.fromCameraInfo(depth_image_info);
  pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
  
  if (first_run_) {
    first_run_ = false;
  } else {
    std::vector<std::vector<Flow3D>> clusters; // clusters[cluster_index][cluster_element_index]
    for (int i = 0; i < optical_flow->flow.size(); i++) {
      opencv_apps::Flow flow = optical_flow->flow[i];
      cv::Point2d uv_now, uv_previous;
      uv_now.x = flow.point.x + flow.velocity.x;
      uv_now.y = flow.point.y + flow.velocity.y;
      uv_previous.x = flow.point.x;
      uv_previous.y = flow.point.y;
      
      tf2::Vector3 point3d_now, point3d_previous;
      if(!getPoint3D(uv_now.x, uv_now.y, *depth_image_now, point3d_now))
        continue;
      if(!getPoint3D(uv_previous.x, uv_previous.y, depth_image_previous_, point3d_previous))
        continue;
      
      // 以前のフレームを現在のフレームに座標変換
      tf2::Stamped<tf2::Transform> tf_previous2now;
      tf2::fromMsg(*camera_transform, tf_previous2now);
      tf2::Vector3 point3d_previous_transformed = tf_previous2now * point3d_previous;
      
      Flow3D flow3d = Flow3D(point3d_previous_transformed, point3d_now);
      ros::Duration time_between_frames = camera_transform->header.stamp - time_stamp_previous_;
      
      pcl::PointXYZI pcl_point;
      pcl_point.x = flow3d.end.getX();
      pcl_point.y = flow3d.end.getY();
      pcl_point.z = flow3d.end.getZ();
      pcl_point.intensity = flow3d.length() * time_between_frames.toSec();
      pcl_point_cloud.push_back(pcl_point);
      
      /*
      if (flow3d.length() < moving_flow_length_ * time_between_frames.toSec())
        continue;
      
      // まだクラスターが一つも存在していなければ
      if (clusters.size() <= 0) {
        clusters.emplace_back();
        clusters[0].push_back(flow3d);
      } else {
        bool already_clustered = false;
        std::vector<std::vector<Flow3D>>::iterator belonged_cluster_it;
        for (auto cluster_it = clusters.begin();  cluster_it != clusters.end(); cluster_it++) {
          for (auto& clustered_flow : *cluster_it) {
            if (flow_start_diff_ < (flow3d.start - clustered_flow.start).length())
              continue;
            if (flow_length_diff_ < abs(flow3d.length() - clustered_flow.length()))
              continue;
            if (flow_radian_diff_ < flow3d.radian2otherFlow(clustered_flow))
              continue;
            
            if (!already_clustered) {
              cluster_it->push_back(flow3d);
              already_clustered = true;
              belonged_cluster_it = cluster_it;
            } else {
              belonged_cluster_it->insert(belonged_cluster_it->end(), cluster_it->begin(), cluster_it->end());
              clusters.erase(cluster_it);
            }
            
            break;
          }
        }
      }
      */
      
      
    }
    
    /*
    pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
    for (int i = 0; i < clusters.size(); i++) {
      for (auto& clustered_flow : clusters[i]) {
        pcl::PointXYZI point_clustered;
        
        point_clustered.x = clustered_flow.end.getX();
        point_clustered.y = clustered_flow.end.getY();
        point_clustered.z = clustered_flow.end.getZ();
        point_clustered.intensity = i;
        
        pcl_point_cloud.push_back(point_clustered);
      }
    }
    */
    
    
    sensor_msgs::PointCloud2 msg_point_cloud;
    pcl::toROSMsg(pcl_point_cloud, msg_point_cloud);
    msg_point_cloud.header.frame_id = depth_image_info->header.frame_id;
    msg_point_cloud.header.stamp = depth_image_info->header.stamp;
    
    point_cloud_pub_.publish(msg_point_cloud);
  }
  depth_image_previous_ = *depth_image_now;
  time_stamp_previous_ = camera_transform->header.stamp;
  
  input_synchronizer_->publish();
}

template<typename T>
bool MovingObjectDetector::getPoint3D_internal(int u, int v, const sensor_msgs::Image& depth_image, tf2::Vector3& point3d)
{
  // Use correct principal point from calibration
  float center_x = camera_model_.cx();
  float center_y = camera_model_.cy();
  
  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  float constant_x = unit_scaling / camera_model_.fx();
  float constant_y = unit_scaling / camera_model_.fy();
  
  const T* depth_row = reinterpret_cast<const T*>(&(depth_image.data[0]));
  int row_step = depth_image.step / sizeof(T);
  depth_row += v * row_step;
  T depth = depth_row[u];
  
  // Missing points denoted by NaNs
  if (!DepthTraits<T>::valid(depth))
    return false;
  
  point3d.setX((u - center_x) * depth * constant_x);
  point3d.setY((v - center_y) * depth * constant_y);
  point3d.setZ(DepthTraits<T>::toMeters(depth));
  
  return true;
}

bool MovingObjectDetector::getPoint3D(int u, int v, const sensor_msgs::Image& depth_image, tf2::Vector3& point3d)
{
  if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    return getPoint3D_internal<uint16_t>(u, v, depth_image, point3d);
  }
  else if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    return getPoint3D_internal<float>(u, v, depth_image, point3d);
  }
  else
  {
    ROS_ERROR("unsupported encoding [%s]", depth_image.encoding.c_str());
    return false;
  }
}

MovingObjectDetector::InputSynchronizer::InputSynchronizer(ros::NodeHandle& node_handle)
{
  image_transport_ = std::make_shared<image_transport::ImageTransport>(node_handle);
  
  std::string publish_depth_image_topic = node_handle.resolveName("synchronizer_output_depth_image");
  std::string publish_left_rect_image_topic = node_handle.resolveName("synchronizer_output_left_rect_image");
  std::string publish_right_rect_image_topic = node_handle.resolveName("synchronizer_output_right_rect_image");
  depth_image_pub_ = image_transport_->advertiseCamera(publish_depth_image_topic, 1);
  left_rect_image_pub_ = image_transport_->advertiseCamera(publish_left_rect_image_topic, 1);
  right_rect_image_pub_ = image_transport_->advertiseCamera(publish_right_rect_image_topic, 1);
  
  std::string subscribe_depth_image_topic = node_handle.resolveName("synchronizer_input_depth_image");
  std::string subscribe_left_rect_image_topic = node_handle.resolveName("synchronizer_input_left_rect_image");
  std::string subscribe_right_rect_image_topic = node_handle.resolveName("synchronizer_input_right_rect_image");
  depth_image_sub_.subscribe(node_handle, subscribe_depth_image_topic, 10);
  depth_info_sub_.subscribe(node_handle, image_transport::getCameraInfoTopic(subscribe_depth_image_topic), 10);
  left_rect_image_sub_.subscribe(*image_transport_, subscribe_left_rect_image_topic, 10);
  left_rect_info_sub_.subscribe(node_handle, image_transport::getCameraInfoTopic(subscribe_left_rect_image_topic), 10);
  right_rect_image_sub_.subscribe(*image_transport_, subscribe_right_rect_image_topic, 10);
  right_rect_info_sub_.subscribe(node_handle, image_transport::getCameraInfoTopic(subscribe_right_rect_image_topic), 10);
  time_sync_ = std::make_shared<DataTimeSynchronizer>(depth_image_sub_, depth_info_sub_, left_rect_image_sub_, left_rect_info_sub_, right_rect_image_sub_, right_rect_info_sub_, 10);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::InputSynchronizer::dataCallBack, this, _1, _2, _3, _4, _5, _6));
}

void MovingObjectDetector::InputSynchronizer::dataCallBack(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info)
{
  static int count = 0;
  
  depth_image_ = *depth_image;
  depth_image_info_ = *depth_image_info;
  left_rect_image_ = *left_rect_image;
  left_rect_info_ = *left_rect_info;
  right_rect_image_ = *right_rect_image;
  right_rect_info_ = *right_rect_info;
  
  // MovingObjectDetector内でinput_synchronizer->publish()を行わない限り処理は開始しないので，初期2フレーム分のデータを送信する
  if (count < 2) {
    publish();
    count++;
  } else if ((ros::Time::now() - last_publish_time_).toSec() > 0.5) { // publishが途中で停止した場合には復帰
    publish();
  }
}

void MovingObjectDetector::InputSynchronizer::publish()
{
  depth_image_pub_.publish(depth_image_, depth_image_info_);
  left_rect_image_pub_.publish(left_rect_image_, left_rect_info_);
  right_rect_image_pub_.publish(right_rect_image_, right_rect_info_);
  
  last_publish_time_ = ros::Time::now();
}
