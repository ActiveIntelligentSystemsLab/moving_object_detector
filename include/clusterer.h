#ifndef __HEADER_CLUSTERER__
#define __HEADER_CLUSTERER__

#include <moving_object_detector/ClustererConfig.h>
#include <moving_object_detector/MovingObjectArray.h>

#define PCL_NO_PRECOMPILE

#include "pcl_point_xyz_velocity.h"

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include "color_set.h"
#include "lookup_table.h"

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <utility>
#include <vector>

struct Point2d {
  int u;
  int v;
  Point2d() : u(0), v(0){}
  Point2d(int u, int v) : u(u), v(v){}
};

class Clusterer {
public:
  Clusterer();
  
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher dynamic_objects_pub_;
  ros::Publisher clusters_pub_;
  ros::Subscriber velocity_pc_sub_;
  
  dynamic_reconfigure::Server<moving_object_detector::ClustererConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_detector::ClustererConfig>::CallbackType reconfigure_func_;
  
  // thresholds
  int cluster_size_th_;
  double depth_diff_th_;
  double dynamic_speed_th_;
  int neighbor_distance_th_;

  pcl::PointCloud<pcl::PointXYZVelocity>::Ptr input_pointcloud_;
  std_msgs::Header input_header_;

  std::vector<int> cluster_map_;
  int max_cluster_number_;
  const int NOT_BELONGED_ = -1; // cluster_map_用，クラスタに未所属の点
  // 番号は違っても実際は同じクラスターどうしの対応づけ
  LookupTable lookup_table_;
  ColorSet color_set_; // クラスタ別に色分けするための色セット

  std::vector<bool> dynamic_map_; // 各点が移動点であるかどうかを格納するマップ
  
  void calculateDynamicMap();
  void cluster2Marker(const pcl::PointIndices& cluster_indices, visualization_msgs::Marker& marker, int marker_id);
  void cluster2MovingObject(const pcl::PointIndices& cluster_indices, moving_object_detector::MovingObject& moving_object);
  void clustering(pcl::IndicesClusters &output_indices);
  int& clusterNumber(const Point2d &point);
  void comparePoints(const Point2d &point1, const Point2d &point2);
  void dataCB(const sensor_msgs::PointCloud2ConstPtr &velocity_pc_msg);
  float depthDiff(const Point2d &point1, const Point2d &point2);
  bool isDynamic(const Point2d &point);
  bool isInRange(const Point2d &point);
  const pcl::PointXYZVelocity& point3dAt(const Point2d& point);
  void publishClusters(const pcl::IndicesClusters &clusters);
  void publishMovingObjects(const pcl::IndicesClusters &clusters);
  void reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level);
};

#endif
