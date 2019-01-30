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

void Clusterer::arrangeLookUpTable()
{
  for (int i = 1; i < look_up_table_.size(); i++)
    look_up_table_.at(i) = lookUp(i);
}

int& Clusterer::clusterNumber(const Point2d &point)
{
  return cluster_map_.at(point.v * input_pointcloud_->width + point.u);
}

void Clusterer::clustering(pcl::IndicesClusters &output_indices)
{
  if (cluster_map_.size() != input_pointcloud_->size())
    cluster_map_.resize(input_pointcloud_->size());
  std::fill(cluster_map_.begin(), cluster_map_.end(), NOT_BELONGED_);

  look_up_table_.clear();
  max_cluster_number_ = -1;

  Point2d interest_point;
  // ラスタスキャン
  for (interest_point.v = 0; interest_point.v < input_pointcloud_->height; interest_point.v++)
  {
    for (interest_point.u = 0; interest_point.u < input_pointcloud_->width; interest_point.u++)
    {
      if (!isDynamic(interest_point))
        continue;

      // 4近傍で隣接ピクセルをチェック
      const Point2d above_point(interest_point.u, interest_point.v - 1);
      comparePoints(interest_point, above_point);

      const Point2d left_point(interest_point.u - 1, interest_point.v);
      comparePoints(interest_point, left_point);
    }
  }

  arrangeLookUpTable();

  // lookup talbeをindices clustersに変換
  for (interest_point.u = 0; interest_point.u < input_pointcloud_->width; interest_point.u++)
  {
    for (interest_point.v = 0; interest_point.v < input_pointcloud_->height; interest_point.v++)
    {
      int temporary_cluster = clusterNumber(interest_point);
      if (temporary_cluster == NOT_BELONGED_)
        continue;

      int true_cluster = look_up_table_.at(temporary_cluster);
      if (output_indices.size() < true_cluster + 1)
        output_indices.resize(true_cluster + 1);
      
      int pointcloud_indice = input_pointcloud_->width * interest_point.v + interest_point.u;
      output_indices.at(true_cluster).indices.push_back(pointcloud_indice);
    }
  }
}

void Clusterer::cluster2Marker(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_cluster, visualization_msgs::Marker& marker, int marker_id)
{
  marker.header = input_header_;
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;

  marker.points.resize(input_cluster->size());
  for (int i = 0; i < input_cluster->size(); i++)
  {
    geometry_msgs::Point point;
    point.x = input_cluster->at(i).x;
    point.y = input_cluster->at(i).y;
    point.z = input_cluster->at(i).z;
    marker.points.at(i) = point;
  }
}

void Clusterer::cluster2movingObject(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_cluster, moving_object_detector::MovingObject& output_moving_object)
{
  Eigen::Vector4f min_point, max_point;
  pcl::getMinMax3D(*input_cluster, min_point, max_point);
  Eigen::Vector4f bounding_box_size = max_point - min_point;
  output_moving_object.bounding_box.x = bounding_box_size(0);
  output_moving_object.bounding_box.y = bounding_box_size(1);
  output_moving_object.bounding_box.z = bounding_box_size(2);

  Eigen::Vector4f center_point = (min_point + max_point) / 2;
  output_moving_object.center.x = center_point(0);
  output_moving_object.center.y = center_point(1);
  output_moving_object.center.z = center_point(2);

  Eigen::Vector3f velocity_sum(0.0, 0.0, 0.0);
  for (auto& point : *input_cluster)
  {
    Eigen::Map<Eigen::Vector3f> velocity(point.data_velocity);
    velocity_sum += velocity;
  }
  int cluster_size = input_cluster->points.size();
  output_moving_object.velocity.x = velocity_sum(0) / cluster_size;
  output_moving_object.velocity.y = velocity_sum(1) / cluster_size;
  output_moving_object.velocity.z = velocity_sum(2) / cluster_size;
}

void Clusterer::comparePoints(const Point2d &interest_point, const Point2d &neighbor_point)
{
  if (!isInRange(neighbor_point))
    return;

  if (!isDynamic(neighbor_point))
    return;

  if (depthDiff(interest_point, neighbor_point) > depth_diff_th_)
    return;
  
  int &interest_point_cluster = clusterNumber(interest_point);
  int &neighbor_point_cluster = clusterNumber(neighbor_point);
  
  if (interest_point_cluster == NOT_BELONGED_ && neighbor_point_cluster == NOT_BELONGED_)
  {
    max_cluster_number_++;
    look_up_table_.push_back(max_cluster_number_);

    interest_point_cluster = max_cluster_number_;
    neighbor_point_cluster = max_cluster_number_;
  }
  else if (interest_point_cluster != NOT_BELONGED_ && neighbor_point_cluster == NOT_BELONGED_)
  {
    neighbor_point_cluster = interest_point_cluster;
  }
  else if (interest_point_cluster == NOT_BELONGED_ && neighbor_point_cluster != NOT_BELONGED_)
  {
    interest_point_cluster = neighbor_point_cluster;
  }
  else if (interest_point_cluster != NOT_BELONGED_ && neighbor_point_cluster != NOT_BELONGED_)
  {
    // 2つのクラスタが合わさって同一のクラスタを構成することを記録しておく
    updateLookUpTable(interest_point_cluster, neighbor_point_cluster);
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
  
  moving_object_detector::MovingObjectArray moving_objects_msg;
  moving_objects_msg.header = input_header_;

  visualization_msgs::MarkerArray clusters_msg;
  visualization_msgs::Marker delete_marker; // 前フレームのmarkerを全て消すように指示を出すマーカー
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  clusters_msg.markers.push_back(delete_marker);
  int marker_id = 0;

  for (auto cluster_it = clusters.begin (); cluster_it != clusters.end (); cluster_it++)
  {
    if (cluster_it->indices.size() <= cluster_size_th_)
      continue;
    
    pcl::PointCloud<pcl::PointXYZVelocity>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZVelocity>);
    indices2cloud(*cluster_it, cluster);

    moving_object_detector::MovingObject moving_object;
    cluster2movingObject(cluster, moving_object);
    moving_objects_msg.moving_object_array.push_back(moving_object);

    visualization_msgs::Marker cluster_marker;
    cluster2Marker(cluster, cluster_marker, marker_id);
    marker_id++;
    clusters_msg.markers.push_back(cluster_marker);
  }
  
  dynamic_objects_pub_.publish(moving_objects_msg);
  clusters_pub_.publish(clusters_msg);

  ros::Duration process_time = ros::Time::now() - start;
  ROS_INFO_STREAM("Process time: " << process_time.toSec() << " [s]");
}

float Clusterer::depthDiff(const Point2d &point1, const Point2d &point2)
{
  return std::abs(point3dAt(point1).z - point3dAt(point2).z);
}

bool Clusterer::isDynamic(const Point2d &point) {
  pcl::PointXYZVelocity point3d = point3dAt(point);
  Eigen::Vector3f velocity(point3d.vx, point3d.vy, point3d.vz);

  if (velocity.norm() > dynamic_speed_th_)
    return true;
  
  return false;
}

bool Clusterer::isInRange(const Point2d &point) {
  if (point.u < 0 || point.u >= input_pointcloud_->width)
    return false;
  
  if (point.v < 0 || point.v >= input_pointcloud_->height)
    return false;

  return true;
}

void Clusterer::indices2cloud(pcl::PointIndices &input_indices, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &output_pc)
{
  output_pc->resize(input_indices.indices.size());
  for (int i = 0; i < input_indices.indices.size(); i++)
  {
    int indice = input_indices.indices.at(i);
    output_pc->points.at(i) = input_pointcloud_->points.at(indice);
  }
  output_pc->width = output_pc->points.size();
  output_pc->height = 1;
  output_pc->is_dense = true;
}


// lookUpテーブルを再帰的に探索し，最終的に適用されるべきクラスタ番号を返す
int Clusterer::lookUp(int cluster)
{
  if (cluster == look_up_table_.at(cluster))
    return cluster;

  return lookUp(look_up_table_.at(cluster));
}

const pcl::PointXYZVelocity &Clusterer::point3dAt(const Point2d &point)
{
  return input_pointcloud_->at(point.u, point.v);
}

void Clusterer::reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level) 
{
  ROS_INFO("Reconfigure Request: cluster_size = %d, dynamic_speed = %f", config.cluster_size, config.dynamic_speed);
  cluster_size_th_  = config.cluster_size;
  depth_diff_th_ = config.depth_diff;
  dynamic_speed_th_ = config.dynamic_speed;
}

void Clusterer::updateLookUpTable(int cluster1, int cluster2)
{
  int early_cluster = std::min(cluster1, cluster2);
  int late_cluster = std::max(cluster1, cluster2);

  if (early_cluster < look_up_table_.at(late_cluster))
    look_up_table_.at(late_cluster) = early_cluster;
}