#ifndef __HEADER_CLUSTERER__
#define __HEADER_CLUSTERER__

#include <moving_object_detector/ClustererConfig.h>
#include <moving_object_detector/MovingObjectArray.h>

#define PCL_NO_PRECOMPILE

#include "pcl_point_xyz_velocity.h"

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Clusterer {
public:
  Clusterer();
  
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher dynamic_objects_pub_;
  ros::Subscriber velocity_pc_sub_;
  
  dynamic_reconfigure::Server<moving_object_detector::ClustererConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_detector::ClustererConfig>::CallbackType reconfigure_func_;
  
  // thresholds
  int cluster_size_th_;
  double dynamic_speed_th_;
  double position_diff_th_;
  // Used in conditionFunction() which is static function
  static double direction_diff_th_;
  static double speed_diff_th_;
  
  void cluster2movingObject(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_cluster, moving_object_detector::MovingObject& output_moving_object);
  void clustering(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, pcl::IndicesClusters &output_indices);
  void dataCB(const sensor_msgs::PointCloud2ConstPtr &velocity_pc_msg);
  void indices2cloud(pcl::PointIndices &input_indices, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &output_pc);
  void reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level);
  void removeStaticPoints(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &velocity_pc);

  // Before PCL 1.8, ConditionalEuclideanClustering.segment() has only raw function pointer as argument.
  static bool conditionFunction(const pcl::PointXYZVelocity& point1, const pcl::PointXYZVelocity& point2, float squared_distance);
};

#endif
