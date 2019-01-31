#include "clusterer.h"

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <cmath>
#include <Eigen/Core>
#include <list>
#include <vector>

Clusterer::Clusterer()
{
  reconfigure_func_ = boost::bind(&Clusterer::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  velocity_pc_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velocity_pc", 10, &Clusterer::dataCB, this);
  dynamic_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("moving_objects", 1);
  clusters_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
}

void Clusterer::calculateDynamicMap()
{
  if (dynamic_map_.size() != input_pointcloud_->size())
    dynamic_map_.resize(input_pointcloud_->size());
  std::fill(dynamic_map_.begin(), dynamic_map_.end(), false);

  for (int i = 0; i < input_pointcloud_->size(); i++)
  {
    pcl::PointXYZVelocity &point3d = input_pointcloud_->at(i);
    Eigen::Vector3f velocity(point3d.vx, point3d.vy, point3d.vz);

    if (velocity.norm() >= dynamic_speed_th_)
      dynamic_map_.at(i) = true;
  }
}

void Clusterer::calculateInitialClusterMap()
{
  lookup_table_.clear();

  Point2d interest_point;
  for (interest_point.v = 0; interest_point.v < input_pointcloud_->height; interest_point.v++)
  {
    for (interest_point.u = 0; interest_point.u < input_pointcloud_->width; interest_point.u++)
    {
      if (!isDynamic(interest_point))
        continue;

      for (int dv = -neighbor_distance_th_; dv <= 0; dv++)
      {
        for (int du = -neighbor_distance_th_; du <= 0; du++)
        {
          Point2d compared_point(interest_point.u + du, interest_point.v + dv);
          comparePoints(interest_point, compared_point);
        }
      }
    }
  }
}

void Clusterer::clustering(pcl::IndicesClusters &output_indices)
{
  calculateDynamicMap();
  initClusterMap();

  calculateInitialClusterMap();
  integrateConnectedClusters();
  removeSmallClusters();

  clusterMap2IndicesCluster(output_indices);
}

void Clusterer::clusterMap2IndicesCluster(pcl::IndicesClusters &indices_clusters)
{
  if (number_of_clusters_ <= 0)
    return;
  
  indices_clusters.resize(number_of_clusters_);
  Point2d point;
  for (point.u = 0; point.u < input_pointcloud_->width; point.u++)
  {
    for (point.v = 0; point.v < input_pointcloud_->height; point.v++)
    {
      int cluster_number = clusterAt(point);
      if (cluster_number == NOT_BELONGED_)
        continue;
      pcl::PointIndices &cluster = indices_clusters.at(cluster_number);
      
      int pointcloud_indice = input_pointcloud_->width * point.v + point.u;
      cluster.indices.push_back(pointcloud_indice);
    }
  }
}

void Clusterer::cluster2Marker(const pcl::PointIndices& cluster_indices, visualization_msgs::Marker& marker, int marker_id)
{
  int r, g, b;
  color_set_.getColor(marker_id, r, g, b);
  marker.header = input_header_;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0;
  marker.color.a = 1.0;
  marker.color.r = r / 255.0;
  marker.color.g = g / 255.0;
  marker.color.b = b / 255.0;

  marker.points.resize(cluster_indices.indices.size());
  for (int i = 0; i < cluster_indices.indices.size(); i++)
  {
    geometry_msgs::Point point;
    int indice = cluster_indices.indices.at(i);
    point.x = input_pointcloud_->at(indice).x;
    point.y = input_pointcloud_->at(indice).y;
    point.z = input_pointcloud_->at(indice).z;
    marker.points.at(i) = point;
  }
}

void Clusterer::cluster2MovingObject(const pcl::PointIndices& cluster_indices, moving_object_detector::MovingObject& moving_object)
{
  pcl::PointCloud<pcl::PointXYZVelocity> cluster(*input_pointcloud_, cluster_indices.indices);

  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);
  Eigen::Vector4f bounding_box_size = max_point - min_point;
  moving_object.bounding_box.x = bounding_box_size(0);
  moving_object.bounding_box.y = bounding_box_size(1);
  moving_object.bounding_box.z = bounding_box_size(2);

  Eigen::Vector4f center_point = (min_point + max_point) / 2;
  moving_object.center.x = center_point(0);
  moving_object.center.y = center_point(1);
  moving_object.center.z = center_point(2);

  Eigen::Vector3f velocity_sum(0.0, 0.0, 0.0);
  for (auto& point : cluster)
  {
    Eigen::Map<Eigen::Vector3f> velocity(point.data_velocity);
    velocity_sum += velocity;
  }
  int cluster_size = cluster.points.size();
  moving_object.velocity.x = velocity_sum(0) / cluster_size;
  moving_object.velocity.y = velocity_sum(1) / cluster_size;
  moving_object.velocity.z = velocity_sum(2) / cluster_size;
}

void Clusterer::comparePoints(const Point2d &interest_point, const Point2d &compared_point)
{
  if (!isInRange(compared_point))
    return;

  if (!isDynamic(compared_point))
    return;

  if (depthDiff(interest_point, compared_point) > depth_diff_th_)
    return;
  
  int &point1_cluster = clusterAt(interest_point);
  int &point2_cluster = clusterAt(compared_point);
  
  if (point1_cluster == NOT_BELONGED_ && point2_cluster == NOT_BELONGED_)
  {
    number_of_clusters_++;
    lookup_table_.resize(number_of_clusters_);

    // クラスタ番号は0から始まるので，-1
    point1_cluster = number_of_clusters_ - 1;
    point2_cluster = number_of_clusters_ - 1;
  }
  else if (point1_cluster != NOT_BELONGED_ && point2_cluster == NOT_BELONGED_)
  {
    point2_cluster = point1_cluster;
  }
  else if (point1_cluster == NOT_BELONGED_ && point2_cluster != NOT_BELONGED_)
  {
    point1_cluster = point2_cluster;
  }
  else if (point1_cluster != NOT_BELONGED_ && point2_cluster != NOT_BELONGED_)
  {
    int source_cluster = std::max(point1_cluster, point2_cluster);
    int destination_cluster = std::min(point1_cluster, point2_cluster);
    // 2つのクラスタが合わさって同一のクラスタを構成することを記録しておく
    lookup_table_.update(source_cluster, destination_cluster);
  }
}

void Clusterer::dataCB(const sensor_msgs::PointCloud2ConstPtr& input_pc_msg)
{
  ros::Time start = ros::Time::now();

  input_pointcloud_ = pcl::PointCloud<pcl::PointXYZVelocity>::Ptr(new pcl::PointCloud<pcl::PointXYZVelocity>);
  pcl::fromROSMsg(*input_pc_msg, *input_pointcloud_);

  input_header_ = input_pc_msg->header;

  pcl::IndicesClusters clusters;
  clustering(clusters);

  if (clusters_pub_.getNumSubscribers() > 0)
    publishClusters(clusters);
  if (dynamic_objects_pub_.getNumSubscribers() > 0)
    publishMovingObjects(clusters);

  ros::Duration process_time = ros::Time::now() - start;
  ROS_INFO_STREAM("Process time: " << process_time.toSec() << " [s]");
}

void Clusterer::initClusterMap()
{
  number_of_clusters_ = 0;

  if (cluster_map_.size() != input_pointcloud_->size())
    cluster_map_.resize(input_pointcloud_->size());
  std::fill(cluster_map_.begin(), cluster_map_.end(), NOT_BELONGED_);
}

void Clusterer::integrateConnectedClusters()
{
  lookup_table_.arrange();

  number_of_clusters_ = 0;
  for (int i = 0; i < cluster_map_.size(); i++)
  {
    int initial_cluster = cluster_map_.at(i);
    if (initial_cluster == NOT_BELONGED_)
      continue;
    
    int final_cluster = lookup_table_.lookup(initial_cluster);
    cluster_map_.at(i) = final_cluster;
    if (final_cluster > number_of_clusters_ - 1)
      number_of_clusters_ = final_cluster + 1;
  }
}

void Clusterer::publishClusters(const pcl::IndicesClusters &clusters)
{
  visualization_msgs::MarkerArray clusters_msg;

  visualization_msgs::Marker delete_marker; // 前フレームのmarkerを全て消すように指示を出すマーカー
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  clusters_msg.markers.push_back(delete_marker);

  int marker_id = 0;

  color_set_.resize(clusters.size());

  for (auto cluster_it = clusters.begin (); cluster_it != clusters.end (); cluster_it++)
  {
    visualization_msgs::Marker cluster_marker;
    cluster2Marker(*cluster_it, cluster_marker, marker_id);
    marker_id++;
    clusters_msg.markers.push_back(cluster_marker);
  }

  clusters_pub_.publish(clusters_msg);
}

void Clusterer::publishMovingObjects(const pcl::IndicesClusters &clusters)
{
  moving_object_detector::MovingObjectArray moving_objects_msg;
  moving_objects_msg.header = input_header_;

  for (auto cluster_it = clusters.begin (); cluster_it != clusters.end (); cluster_it++)
  {
    moving_object_detector::MovingObject moving_object;
    cluster2MovingObject(*cluster_it, moving_object);
    moving_objects_msg.moving_object_array.push_back(moving_object);
  }

  dynamic_objects_pub_.publish(moving_objects_msg);
}

void Clusterer::reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level) 
{
  ROS_INFO("Reconfigure Request: cluster_size = %d, depth_diff %f, dynamic_speed = %f, neighbor_distance = %d", config.cluster_size, config.depth_diff, config.dynamic_speed, config.neighbor_distance);
  cluster_size_th_  = config.cluster_size;
  depth_diff_th_ = config.depth_diff;
  dynamic_speed_th_ = config.dynamic_speed;
  neighbor_distance_th_ = config.neighbor_distance;
}

void Clusterer::removeSmallClusters()
{
  if (number_of_clusters_ <= 0)
    return;
  // 各クラスタの要素数を計算
  std::vector<size_t> cluster_size(number_of_clusters_, 0);
  for (int i = 0; i < cluster_map_.size(); i++)
  {
    int cluster = cluster_map_.at(i);
    if (cluster == NOT_BELONGED_)
      continue;

    cluster_size.at(cluster) += 1;
  }

  // 削除するクラスターの特定
  // また，クラスタの削除によってクラスタ番号が断続的になるので，連番に修正する
  std::vector<int> cluster_old2new(number_of_clusters_);
  for (int i = 0; i < cluster_size.size(); i++)
  {
    if (cluster_size.at(i) < cluster_size_th_)
    {
      cluster_old2new.at(i) = NOT_BELONGED_;
      number_of_clusters_--;
    }
    else
    {
      cluster_old2new.at(i) = i - (cluster_size.size() - number_of_clusters_);
    }
  }

  // 所属クラスタの更新
  for (int i = 0; i < cluster_map_.size(); i++)
  {
    int old_cluster = cluster_map_.at(i); 
    if (old_cluster == NOT_BELONGED_)
      continue;
    cluster_map_.at(i) = cluster_old2new.at(old_cluster);
  }
}