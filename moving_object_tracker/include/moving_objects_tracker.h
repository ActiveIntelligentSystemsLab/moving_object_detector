#ifndef __HEADER_MOVING_OBJECTS_TRACKER__
#define __HEADER_MOVING_OBJECTS_TRACKER__

#include <kkl/alg/data_association.hpp>
#include <moving_object_msgs/MovingObjectArray.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "kalman_tracker.hpp"

#include <memory>
#include <vector>

class MovingObjectsTracker
{
public:
  MovingObjectsTracker();
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber moving_objects_sub_;
  ros::Publisher tracked_moving_objects_pub_;
  ros::Publisher trackers_covariance_pub_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<KalmanTracker::Ptr> trackers;
  std::shared_ptr<kkl::alg::DataAssociation<KalmanTracker::Ptr, moving_object_msgs::MovingObjectPtr>> data_association;

  double object_radius_;
  double covariance_trace_limit_;
  int id_gen_;

  void movingObjectsCallback(const moving_object_msgs::MovingObjectArrayConstPtr &moving_objects);
  void predict(const ros::Time& time);
  void correct(const ros::Time& time, const std::vector<moving_object_msgs::MovingObjectPtr> &moving_objects);
};

#endif
