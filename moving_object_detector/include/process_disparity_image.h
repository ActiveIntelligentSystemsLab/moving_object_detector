#ifndef __HEADER_PROCESS_DISPARITY_IMAGE__
#define HEADADER_PROCESS_DISPARITY_IMAGE__

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf2/LinearMath/Vector3.h>

#include <exception>
#include <opencv2/core/core.hpp>

class ProcessDisparityImage
{
public:
  ProcessDisparityImage(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::CameraInfoConstPtr& left_camera_info);
  
  bool getDisparity(int u, int v, float& disparity);
  bool getPoint3D(int u, int v, tf2::Vector3& point3d);
  int getWidth();
  int getHeight();
private:
  image_geometry::PinholeCameraModel _left_camera_model;
  
  stereo_msgs::DisparityImageConstPtr _disparity_msg;
  cv::Mat _disparity_map;
};


#endif // __HEADER_PROCESS_DISPARITY_IMAGE__
