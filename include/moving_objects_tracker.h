#ifndef __HEADER_MOVING_OBJECTS_TRACKER__
#define __HEADER_MOVING_OBJECTS_TRACKER__

#include <moving_object_detector/MovingObjectArray.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// Koideさん作のやつ
#include <kkl/alg/data_association.hpp>
// Koideさんのやつをベースにした何か
#include <kalman_tracker.hpp>

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

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<KalmanTracker::Ptr> trackers;
  std::vector<KalmanTracker::Ptr> removed_objects;
  std::shared_ptr<kkl::alg::DataAssociation<KalmanTracker::Ptr, moving_object_detector::MovingObjectPtr>> data_association;

  double object_radius_;
  double covariance_trace_limit_;
  int id_gen_;

  void movingObjectsCallback(const moving_object_detector::MovingObjectArrayConstPtr &moving_objects);
  void predict(const ros::Time& time);
  void correct(const ros::Time& time, const std::vector<moving_object_detector::MovingObjectPtr> &moving_objects);
};

#endif
