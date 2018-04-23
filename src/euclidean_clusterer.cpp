#include "euclidean_clusterer.h"

#define PCL_NO_PRECOMPILE
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <Eigen/Core>

EuclideanClusterer::EuclideanClusterer()
{
  reconfigure_func_ = boost::bind(&EuclideanClusterer::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);

  velocity_pc_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velocity_pc", 10, &EuclideanClusterer::dataCB, this);
  dynamic_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("moving_objects", 10);
}

void EuclideanClusterer::cluster2movingObject(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr& input_cluster, moving_object_detector::MovingObject& output_moving_object)
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

void EuclideanClusterer::dataCB(const sensor_msgs::PointCloud2ConstPtr& velocity_pc_msg)
{
  pcl::PointCloud<pcl::PointXYZVelocity>::Ptr velocity_pc(new pcl::PointCloud<pcl::PointXYZVelocity>);
  pcl::fromROSMsg(*velocity_pc_msg, *velocity_pc);

  removeStaticPoints(velocity_pc);

  std::vector<pcl::PointIndices> cluster_indices;
  euclideanClustering(velocity_pc, cluster_indices);

  moving_object_detector::MovingObjectArray pub_msg;
  pub_msg.header = velocity_pc_msg->header;
  for (auto cluster_it = cluster_indices.begin (); cluster_it != cluster_indices.end (); cluster_it++)
  {
    pcl::PointCloud<pcl::PointXYZVelocity>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZVelocity>);
    indices2cloud(*cluster_it, velocity_pc, cluster);

    moving_object_detector::MovingObject moving_object;
    cluster2movingObject(cluster, moving_object);

    pub_msg.moving_object_array.push_back(moving_object);
  }

  dynamic_objects_pub_.publish(pub_msg);
}

void EuclideanClusterer::euclideanClustering(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, std::vector<pcl::PointIndices> &output_indices)
{
  pcl::search::KdTree<pcl::PointXYZVelocity>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZVelocity>);
  tree->setInputCloud(input_pc);

  pcl::EuclideanClusterExtraction<pcl::PointXYZVelocity> ec;
  ec.setClusterTolerance(neighbor_distance_th_);
  ec.setMinClusterSize(cluster_size_th_);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_pc);
  ec.extract(output_indices);
}

void EuclideanClusterer::indices2cloud(pcl::PointIndices &input_indices, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &output_pc)
{
  for (auto indice_it = input_indices.indices.begin(); indice_it != input_indices.indices.end(); indice_it++)
    output_pc->points.push_back(input_pc->points[*indice_it]);
  output_pc->width = output_pc->points.size();
  output_pc->height = 1;
  output_pc->is_dense = true;
}

void EuclideanClusterer::reconfigureCB(moving_object_detector::EuclideanClustererConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: cluster_size = %d, dynamic_speed = %f, neighbor_distance = %f", config.cluster_size, config.dynamic_speed, config.neighbor_distance);

  cluster_size_th_      = config.cluster_size;
  dynamic_speed_th_     = config.dynamic_speed;
  neighbor_distance_th_ = config.neighbor_distance;
}

void EuclideanClusterer::removeStaticPoints(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &velocity_pc)
{
  for (auto point_it = velocity_pc->begin(); point_it != velocity_pc->end(); point_it++)
  {
    Eigen::Map<Eigen::Vector3f> velocity(point_it->data_velocity);
    if (velocity.norm() < dynamic_speed_th_)
    {
      point_it = velocity_pc->erase(point_it);
      point_it--;
    }
  }
}
