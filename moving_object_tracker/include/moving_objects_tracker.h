#ifndef MOVING_OBJECTS_TRACKER_H_
#define MOVING_OBJECTS_TRACKER_H_

#include <dynamic_reconfigure/server.h>
#include <kkl/alg/data_association.hpp>
#include <moving_object_msgs/MovingObjectArray.h>
#include <moving_object_tracker/MovingObjectsTrackerConfig.h>
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
  std::shared_ptr<ros::NodeHandle> private_node_handle_;
  ros::Subscriber moving_objects_sub_;
  ros::Publisher tracked_moving_objects_pub_;
  ros::Publisher trackers_covariance_pub_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<KalmanTracker::Ptr> trackers;
  std::shared_ptr<kkl::alg::DataAssociation<KalmanTracker::Ptr, moving_object_msgs::MovingObjectPtr>> data_association;

  std::string odom_frame_id_;
  double object_radius_;
  double covariance_trace_limit_;
  int correction_count_limit_;
  int id_gen_;
  double gating_mahalanobis_;
  double gating_deviation_;

  dynamic_reconfigure::Server<moving_object_tracker::MovingObjectsTrackerConfig> reconfigure_server_;
  dynamic_reconfigure::Server<moving_object_tracker::MovingObjectsTrackerConfig>::CallbackType reconfigure_function_;

  void reconfigureCallback(moving_object_tracker::MovingObjectsTrackerConfig &config, uint32_t level);
  void movingObjectsCallback(const moving_object_msgs::MovingObjectArrayConstPtr &moving_objects);
  void predict(const ros::Time& time);
  void correct(const ros::Time& time, const std::vector<moving_object_msgs::MovingObjectPtr> &moving_objects);
};

#endif
