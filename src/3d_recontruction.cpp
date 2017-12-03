#include "3d_recontruction.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
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

template<typename T>
bool getPoint3D_internal(int u, int v, image_geometry::PinholeCameraModel& camera_model, const sensor_msgs::Image& depth_image, tf2::Vector3& point3d)
{
  // Use correct principal point from calibration
  float center_x = camera_model.cx();
  float center_y = camera_model.cy();
  
  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  float constant_x = unit_scaling / camera_model.fx();
  float constant_y = unit_scaling / camera_model.fy();
  
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

bool getPoint3D(int u, int v, image_geometry::PinholeCameraModel& camera_model, const sensor_msgs::Image& depth_image, tf2::Vector3& point3d)
{
  if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    return getPoint3D_internal<uint16_t>(u, v, camera_model, depth_image, point3d);
  }
  else if (depth_image.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    return getPoint3D_internal<float>(u, v, camera_model, depth_image, point3d);
  }
  else
  {
    ROS_ERROR("unsupported encoding [%s]", depth_image.encoding.c_str());
    return false;
  }
}
