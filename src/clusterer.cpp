#include "clusterer.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <Eigen/Core>
#include <list>

double Clusterer::direction_diff_th_;
double Clusterer::speed_diff_th_;

Clusterer::Clusterer()
{
  reconfigure_func_ = boost::bind(&Clusterer::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  velocity_pc_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velocity_pc", 10, &Clusterer::dataCB, this);
  dynamic_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("moving_objects", 10);
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

void Clusterer::clustering(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, pcl::IndicesClusters &output_indices)
{
  pcl::search::KdTree<pcl::PointXYZVelocity>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZVelocity>);
  tree->setInputCloud(input_pc);

  pcl::ConditionalEuclideanClustering<pcl::PointXYZVelocity> cec;
  cec.setInputCloud(input_pc);
  cec.setConditionFunction(Clusterer::conditionFunction);
  cec.setClusterTolerance(position_diff_th_);
  cec.setMinClusterSize(cluster_size_th_);
  cec.segment(output_indices);
}

// Before PCL 1.8, ConditionalEuclideanClustering.segment() has only raw function pointer as argument.
bool Clusterer::conditionFunction(const pcl::PointXYZVelocity& point1, const pcl::PointXYZVelocity& point2, float squared_distance)
{
  pcl::PointXYZVelocity tmp_point1, tmp_point2;
  tmp_point1 = point1;
  Eigen::Map<Eigen::Vector3f> velocity1(tmp_point1.data_velocity);
  tmp_point2 = point2;
  Eigen::Map<Eigen::Vector3f> velocity2(tmp_point2.data_velocity);

  if (Clusterer::speed_diff_th_ < std::abs(velocity1.norm() - velocity2.norm()))
    return false;
  if (Clusterer::direction_diff_th_ < std::acos(velocity1.dot(velocity2) / (velocity1.norm() * velocity2.norm())))
    return false;

  return true;
}

void Clusterer::dataCB(const sensor_msgs::PointCloud2ConstPtr& velocity_pc_msg)
{
  pcl::PointCloud<pcl::PointXYZVelocity>::Ptr velocity_pc(new pcl::PointCloud<pcl::PointXYZVelocity>);
  pcl::fromROSMsg(*velocity_pc_msg, *velocity_pc);

  removeStaticPoints(velocity_pc);

  pcl::IndicesClusters clusters;
  clustering(velocity_pc, clusters);
  
  moving_object_detector::MovingObjectArray pub_msg;
  pub_msg.header = velocity_pc_msg->header;
  for (auto cluster_it = clusters.begin (); cluster_it != clusters.end (); cluster_it++)
  {
    pcl::PointCloud<pcl::PointXYZVelocity>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZVelocity>);
    indices2cloud(*cluster_it, velocity_pc, cluster);

    moving_object_detector::MovingObject moving_object;
    cluster2movingObject(cluster, moving_object);

    pub_msg.moving_object_array.push_back(moving_object);
  }
  
  dynamic_objects_pub_.publish(pub_msg);
}

void Clusterer::indices2cloud(pcl::PointIndices &input_indices, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &input_pc, pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &output_pc)
{
  for (auto indice_it = input_indices.indices.begin(); indice_it != input_indices.indices.end(); indice_it++)
    output_pc->points.push_back(input_pc->points[*indice_it]);
  output_pc->width = output_pc->points.size();
  output_pc->height = 1;
  output_pc->is_dense = true;
}

void Clusterer::reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: cluster_size = %d, direction_diff = %f, dynamic_speed = %f, position_diff = %f, speed_diff = %f", config.cluster_size, config.direction_diff, config.dynamic_speed, config.position_diff, config.speed_diff);
  
  cluster_size_th_   = config.cluster_size;
  direction_diff_th_ = config.direction_diff;
  dynamic_speed_th_  = config.dynamic_speed;
  position_diff_th_  = config.position_diff;
  speed_diff_th_     = config.speed_diff;
}

void Clusterer::removeStaticPoints(pcl::PointCloud<pcl::PointXYZVelocity>::Ptr &velocity_pc)
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

