#include "process_disparity_image.h"

ProcessDisparityImage::ProcessDisparityImage(const stereo_msgs::DisparityImageConstPtr& disparity_msg, const sensor_msgs::CameraInfoConstPtr& left_camera_info) : _disparity_msg(disparity_msg)
{
  _disparity_map = cv::Mat_<float>(_disparity_msg->image.height, _disparity_msg->image.width, (float*)&_disparity_msg->image.data[0], _disparity_msg->image.step);
  
  _left_camera_model.fromCameraInfo(left_camera_info);
}

bool ProcessDisparityImage::getDisparity(int u, int v, float& disparity)
{
  if (u < 0 || u >= getWidth())
    return false;
  if (v < 0 || v >= getHeight())
    return false;
  disparity = _disparity_map.at<float>(v, u);
  
  if (_disparity_msg->max_disparity < disparity)
    return false;
  else if (_disparity_msg->min_disparity > disparity)
    return false;
  
  return true;
}

bool ProcessDisparityImage::getPoint3D(int u, int v, pcl::PointXYZ &point3d)
{
  float disparity;
  if (!getDisparity(u, v, disparity))
    return false;
  if (disparity == 0.0)
    return false;

  float focal_length = _disparity_msg->f;
  float baseline = _disparity_msg->T;
  
  point3d.z = focal_length * baseline / disparity;
  cv::Point3d direction_vector = _left_camera_model.projectPixelTo3dRay(cv::Point2d(u, v));
  point3d.x = direction_vector.x * point3d.z;
  point3d.y = direction_vector.y * point3d.z;

  return true;
}

bool ProcessDisparityImage::getPoint3D(int u, int v, tf2::Vector3& point3d)
{
  float disparity;
  if (!getDisparity(u, v, disparity))
    return false;
  if (disparity == 0.0)
    return false;
  float focal_length = _disparity_msg->f;
  float baseline = _disparity_msg->T;
  
  float z = focal_length * baseline / disparity;
  // a vector faces the point in 3D coordinate
  // vector.z == 1.0
  cv::Point3d direction_vector = _left_camera_model.projectPixelTo3dRay(cv::Point2d(u, v));
  float x = direction_vector.x * z;
  float y = direction_vector.y * z;
  
  point3d.setX(x);
  point3d.setY(y);
  point3d.setZ(z);
  
  return true;
}

int ProcessDisparityImage::getWidth()
{
  return _disparity_map.cols;
}

int ProcessDisparityImage::getHeight()
{
  return _disparity_map.rows;
}

void ProcessDisparityImage::toPointCloud(pcl::PointCloud<pcl::PointXYZ> &pointcloud)
{
  float nan = std::nan("");
  pcl::PointXYZ default_value(nan, nan, nan);
  // 画像座標との対応関係を残すため，organizedなpointcloudを構築する
  pointcloud = pcl::PointCloud<pcl::PointXYZ>(getWidth(), getHeight(), default_value);
  pointcloud.is_dense = false;
  
  for (int u = 0; u < getWidth(); u++)
  {
    for (int v = 0; v < getHeight(); v++)
    {
      pcl::PointXYZ point;
      if (getPoint3D(u, v, point))
        pointcloud.at(u, v) = point;
    }
  }
}