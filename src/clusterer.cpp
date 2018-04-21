#include "clusterer.h"
#include "pcl_point_xyz_velocity.h"

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <Eigen/Core>
#include <list>

Clusterer::Clusterer()
{
  reconfigure_func_ = boost::bind(&Clusterer::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  velocity_pc_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velocity_pc", 10, &Clusterer::dataCB, this);
  dynamic_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("moving_objects", 10);
}

void Clusterer::dataCB(const sensor_msgs::PointCloud2ConstPtr& velocity_pc_msg)
{  
  std::list<std::list<pcl::PointXYZVelocity>> cluster_list;

  pcl::PointCloud<pcl::PointXYZVelocity> velocity_pc;
  pcl::fromROSMsg(*velocity_pc_msg, velocity_pc);
  for (auto &point : velocity_pc)
  {
    Eigen::Map<Eigen::Vector3f> velocity(point.data_velocity);
    Eigen::Map<Eigen::Vector3f> position(point.data);
    
    if (velocity.norm() < dynamic_speed_th_)
      continue;
    
    bool already_clustered = false;
    std::list<std::list<pcl::PointXYZVelocity>>::iterator belonged_cluster_it;
    for (auto cluster_it = cluster_list.begin(); cluster_it != cluster_list.end(); cluster_it++)
    {
      for (auto& clustered_point : *cluster_it)
      {
        Eigen::Map<Eigen::Vector3f> velocity_clustered(clustered_point.data_velocity);
        Eigen::Map<Eigen::Vector3f> position_clustered(clustered_point.data);
        
        if (position_diff_th_ < (position - position_clustered).norm())
          continue;
        if (speed_diff_th_ < std::abs(velocity.norm() - velocity_clustered.norm()))
          continue;
        if (direction_diff_th_ < std::acos(velocity.dot(velocity_clustered) / (velocity.norm() * velocity_clustered.norm())))
          continue;
        
        if (!already_clustered) {
          cluster_it->push_back(point);
          already_clustered = true;
          belonged_cluster_it = cluster_it;
        } else {
          // クラスタに所属済みの点が，他のクラスタにも属していればクラスタ同士を結合する
          auto tmp_cluster_it = cluster_it;
          cluster_it--; // 現在のクラスタが消去されるので，イテレータを一つ前に戻す 次のループのインクリメントで消去されたクラスタの次のクラスタに到達することになる
          belonged_cluster_it->splice(belonged_cluster_it->end(), *tmp_cluster_it);
          cluster_list.erase(tmp_cluster_it);
        }
        
        break;
      }
    }
    
    // どのクラスタにも振り分けられなければ，新しいクラスタを作成
    if (!already_clustered)
    {
      cluster_list.emplace_back();
      cluster_list.back().push_back(point);
    }
  }
  
  moving_object_detector::MovingObjectArray pub_msg;
  pub_msg.header = velocity_pc_msg->header;
  for (auto cluster_it = cluster_list.begin(); cluster_it != cluster_list.end(); cluster_it++)
  {
    if (cluster_it->size() < cluster_size_th_)
      continue;

    pcl::PointCloud<pcl::PointXYZVelocity> pcl_cluster;

    for (auto cluster_element : *cluster_it)
    {
      pcl_cluster.push_back(cluster_element);
    }
    
    moving_object_detector::MovingObject moving_object;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(pcl_cluster, min_pt, max_pt);
    Eigen::Vector4f bounding_box_size = max_pt - min_pt;
    moving_object.bounding_box.x = bounding_box_size(0);
    moving_object.bounding_box.y = bounding_box_size(1);
    moving_object.bounding_box.z = bounding_box_size(2);

    Eigen::Vector4f center_point = (min_pt + max_pt) / 2;
    moving_object.center.x = center_point(0);
    moving_object.center.y = center_point(1);
    moving_object.center.z = center_point(2);
    
    Eigen::Vector3f velocity_sum(0.0, 0.0, 0.0);
    for (auto& point : *cluster_it)
    {
      Eigen::Map<Eigen::Vector3f> velocity(point.data_velocity);
      velocity_sum += velocity;
    }
    moving_object.velocity.x = velocity_sum(0) / cluster_it->size();
    moving_object.velocity.y = velocity_sum(1) / cluster_it->size();
    moving_object.velocity.z = velocity_sum(2) / cluster_it->size();
    
    pub_msg.moving_object_array.push_back(moving_object);
  }
  
  dynamic_objects_pub_.publish(pub_msg);
}

void Clusterer::reconfigureCB(moving_object_detector::ClustererConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: cluster_size = %d, direction_diff = %f, dynamic_speed = %f, position_diff = %f, speed_diff = %f", config.cluster_size, config.direction_diff, config.dynamic_speed, config.position_diff, config.speed_diff);
  
  cluster_size_th_   = config.cluster_size;
  direction_diff_th_ = config.direction_diff;
  dynamic_speed_th_  = config.dynamic_speed;
  position_diff_th_  = config.position_diff;
  speed_diff_th_     = config.speed_diff;
}
