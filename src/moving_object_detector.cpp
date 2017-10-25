#include "moving_object_detector.h"
#include "flow_3d.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
  
  node_handle_.param("~moving_flow_length", moving_flow_length_, 0.05);
  node_handle_.param("~flow_length_diff", flow_length_diff_, 0.05);
  node_handle_.param("~flow_start_diff", flow_start_diff_, 0.10);
  node_handle_.param("~flow_radian_diff", flow_radian_diff_, 0.17);
  
  point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("clustered_point_cloud", 10);
  
  message_filters::Subscriber<geometry_msgs::TransformStamped> camera_transform_sub;
  message_filters::Subscriber<opencv_apps::FlowArrayStamped> optical_flow_sub;
  image_transport::ImageTransport image_transport(node_handle_);
  image_transport::SubscriberFilter depth_image_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_image_info_sub;
  
  std::string depth_image_topic = node_handle_.resolveName("depth_image_rectified"); // image_transport::SubscriberFilter は何故か名前解決してくれないので
  std::string depth_image_info_topic = image_transport::getCameraInfoTopic(depth_image_topic);
  
  camera_transform_sub.subscribe(node_handle_, "camera_transform", 10);
  optical_flow_sub.subscribe(node_handle_, "optical_flow", 10); // optical flowはrectified imageで計算すること
  depth_image_sub.subscribe(image_transport, depth_image_topic, 10);
  depth_image_info_sub.subscribe(node_handle_, depth_image_info_topic, 10);
  message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, opencv_apps::FlowArrayStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(camera_transform_sub, optical_flow_sub, depth_image_sub, depth_image_info_sub, 10);
  sync.registerCallback(boost::bind(&MovingObjectDetector::dataCB, this, _1, _2, _3, _4));
}

void MovingObjectDetector::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const opencv_apps::FlowArrayStampedConstPtr& optical_flow, const sensor_msgs::ImageConstPtr& depth_image_now, const sensor_msgs::CameraInfoConstPtr& depth_image_info)
{
  camera_model_.fromCameraInfo(depth_image_info);
  
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
      if (flow3d.length() < moving_flow_length_)
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
    }
    
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
    sensor_msgs::PointCloud2 msg_point_cloud;
    pcl::toROSMsg(pcl_point_cloud, msg_point_cloud);
    
    point_cloud_pub_.publish(msg_point_cloud);
  }
  depth_image_previous_ = *depth_image_now;
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
