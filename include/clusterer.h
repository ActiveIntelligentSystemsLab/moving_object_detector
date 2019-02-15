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
#include <image_transport/image_transport.h>
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

  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  
  ros::Publisher dynamic_objects_pub_;
  ros::Publisher clusters_pub_;
  image_transport::Publisher clusters_image_pub_;
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
  int number_of_clusters_; // クラスタの個数
  const int NOT_BELONGED_ = -1; // cluster_map_用，クラスタに未所属の点
  // 番号は違っても実際は同じクラスターどうしの対応づけ
  LookupTable lookup_table_;
  ColorSet color_set_; // クラスタ別に色分けするための色セット

  std::vector<bool> dynamic_map_; // 各点が移動点であるかどうかを格納するマップ
  
  void calculateDynamicMap();
  void calculateInitialClusterMap();
  void cluster2Marker(const pcl::PointIndices& cluster_indices, visualization_msgs::Marker& marker, int marker_id);
  bool cluster2MovingObject(const pcl::PointIndices& cluster_indices, moving_object_detector::MovingObject& moving_object);
  void clustering(pcl::IndicesClusters &output_indices);
  void clusterMap2IndicesCluster(pcl::IndicesClusters &indices_clusters);
  inline int& clusterAt(const Point2d &point)
  {
    return cluster_map_.at(point.v * input_pointcloud_->width + point.u);
  };
  void comparePoints(const Point2d &insterest_point, const Point2d &compared_point);
  void dataCB(const sensor_msgs::PointCloud2ConstPtr &velocity_pc_msg);
  inline float depthDiff(const Point2d &point1, const Point2d &point2)
  {
    return std::abs(point3dAt(point1).z - point3dAt(point2).z);
  };
  void initClusterMap();
  void integrateConnectedClusters();
  inline bool isDynamic(const Point2d &point)
  {
    return dynamic_map_.at(input_pointcloud_->width * point.v + point.u);
  };
  inline bool isInRange(const Point2d &point)
  {
    if (point.u < 0 || point.u >= input_pointcloud_->width || point.v < 0 || point.v >= input_pointcloud_->height)
      return false;
    return true;
  };
  inline const pcl::PointXYZVelocity& point3dAt(const Point2d& point)
  {
      return input_pointcloud_->at(point.u, point.v);
  };
  void publishClusters(const pcl::IndicesClusters &clusters);
  void publishClustersImage();
  void publishMovingObjects(const pcl::IndicesClusters &clusters);
  void reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level);
  void removeSmallClusters();
};

#endif
