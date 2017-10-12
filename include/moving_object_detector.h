#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "viso2_ros/VisoMatchPoints.h"
#include "pcl_ros/point_cloud.h"
#include "image_geometry/stereo_camera_model.h"

class MovingObjectDetector {
public:
  MovingObjectDetector();
private:
  ros::NodeHandle nh;
  
  void dataCB(const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info, const viso2_ros::VisoMatchPointsConstPtr& match_points);
  pcl::PointXYZ compute3dPoint(double left_u, double left_v, double right_u, const image_geometry::StereoCameraModel& camera_model);
};