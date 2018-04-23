#ifndef __HEADER_EUCLIDEAN_CLUSTERER__
#define __HEADER_EUCLIDEAN_CLUSTERER__

#include <moving_object_detector/EuclideanClustererConfig.h>
#include <moving_object_detector/MovingObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include "pcl_point_xyz_velocity.h"

class EuclideanClusterer {
public:
  EuclideanClusterer();

private:
  ros::NodeHandle node_handle_;

  ros::Publisher dynamic_objects_pub_;
  ros::Subscriber velocity_pc_sub_;

  dynamic_reconfigure::Server<moving_object_detector::EuclideanClustererConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_detector::EuclideanClustererConfig>::CallbackType reconfigure_func_;

  // thresholds
  double dynamic_speed_th_;
  int cluster_size_th_;
  double neighbor_distance_th_;

  void cluster2movingObject(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_cluster, moving_object_detector::MovingObject& output_moving_object);
  void dataCB(const sensor_msgs::PointCloud2ConstPtr& velocity_pc_msg);
  void euclideanClustering(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_pc, std::vector<pcl::PointIndices>& output_indices);
  void indices2cloud(pcl::PointIndices& input_indices, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_pc, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& output_pc);
  void reconfigureCB(moving_object_detector::EuclideanClustererConfig& config, uint32_t level);
  void removeStaticPoints(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& velocity_pc);
};

#endif // __HEADER_EUCLIDEAN_CLUSTERER__
