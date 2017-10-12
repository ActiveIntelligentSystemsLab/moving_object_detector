#include "moving_object_detector.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

MovingObjectDetector::MovingObjectDetector() {
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_camera_info_sub;
  message_filters::Subscriber<viso2_ros::VisoMatchPoints> match_points_sub;
  
  match_points_sub.subscribe(nh, "match_points", 10);
  left_camera_info_sub.subscribe(nh, "left_camera_info", 10);
  right_camera_info_sub.subscribe(nh, "right_camera_info", 10);
  message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, viso2_ros::VisoMatchPoints> sync(left_camera_info_sub, right_camera_info_sub, match_points_sub, 10);
  sync.registerCallback(boost::bind(&MovingObjectDetector::dataCB, this, _1, _2, _3));
}

void MovingObjectDetector::dataCB(const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info, const viso2_ros::VisoMatchPointsConstPtr& match_points)
{
  // read calibration info from camera info message
  image_geometry::StereoCameraModel camera_model;
  camera_model.fromCameraInfo(*left_camera_info, *right_camera_info);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZ>());
  pc_current->header.frame_id = left_camera_info->header.frame_id;
  pc_current->header.stamp = pcl_conversions::toPCL(left_camera_info->header).stamp;
  pc_current->width = 1;
  pc_current->height = match_points->points.size();
  pc_current->points.resize(match_points->points.size());
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_previous(new pcl::PointCloud<pcl::PointXYZ>());
  pc_previous->header.frame_id = left_camera_info->header.frame_id;
  pc_previous->header.stamp = pcl_conversions::toPCL(left_camera_info->header).stamp;
  pc_previous->width = 1;
  pc_previous->height = match_points->points.size();
  pc_previous->points.resize(match_points->points.size());
  
  std::vector<double> moved_distance;
  std::vector<double> move_angle;
  
  for (size_t i = 0; i < match_points->points.size(); ++i)
  {
    viso2_ros::VisoMatchPoint match = match_points->points[i];
    
    pc_current->points[i] = compute3dPoint(match.left_current.u, match.left_current.v, match.right_current.u, camera_model);
    pc_previous->points[i] = compute3dPoint(match.left_previous.u, match.left_previous.v, match.right_previous.u, camera_model);  
  }
}

pcl::PointXYZ MovingObjectDetector::compute3dPoint(double left_u, double left_v, double right_u, const image_geometry::StereoCameraModel& camera_model) {
  pcl::PointXYZ point_pcl;
  cv::Point2d left_cv;
  left_cv.x = left_u;
  left_cv.y = left_v;
  cv::Point3d point_3d;
  double disparity = left_u - right_u;
  camera_model.projectDisparityTo3d(left_cv, disparity, point_3d);
  point_pcl.x = point_3d.x;
  point_pcl.y = point_3d.y;
  point_pcl.z = point_3d.z;
  
  return point_pcl;
}