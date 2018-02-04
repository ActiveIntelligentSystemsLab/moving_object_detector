#ifndef __HEADER_CLUSTERER__
#define __HEADER_CLUSTERER__

#include <moving_object_detector/MovingObjectArray.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Clusterer {
public:
  Clusterer();
  
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher dynamic_objects_pub_;
  ros::Subscriber velocity_pc_sub_;
  
  // thresholds
  int cluster_size_th_;
  double direction_diff_th_;
  double dynamic_speed_th_;
  double position_diff_th_;
  double speed_diff_th_;
  
  void dataCB(const sensor_msgs::PointCloud2ConstPtr &velocity_pc_msg);
};

#endif
