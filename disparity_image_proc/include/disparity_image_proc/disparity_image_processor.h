#ifndef DISPARITY_IMAGE_PROCESSOR_H
#define DISPARITY_IMAGE_PROCESSOR_H

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf2/LinearMath/Vector3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <exception>
#include <opencv2/core/core.hpp>

class DisparityImageProcessor
{
public:
  DisparityImageProcessor(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::CameraInfoConstPtr& left_camera_info);
  
  bool getDisparity(int u, int v, float& disparity);
  bool getPoint3D(int u, int v, pcl::PointXYZ& point3d);
  bool getPoint3D(int u, int v, tf2::Vector3& point3d);
  int getWidth();
  int getHeight();
  void toPointCloud(pcl::PointCloud<pcl::PointXYZ> &pointcloud);
private:
  image_geometry::PinholeCameraModel _left_camera_model;
  
  stereo_msgs::DisparityImageConstPtr _disparity_msg;
  cv::Mat _disparity_map;
};


#endif // DISPARITY_IMAGE_PROCESSOR_H
