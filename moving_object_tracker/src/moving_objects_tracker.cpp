#include "moving_objects_tracker.h"

#include <geometry_msgs/TransformStamped.h>
#include <kkl/alg/nearest_neighbor_association.hpp>
#include <moving_object_tracker/TrackerCovarianceArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace kkl {
  namespace alg {

/**
 * @brief definition of the distance between tracker and observation for data association
 */
template<>
boost::optional<double> distance(const std::shared_ptr<KalmanTracker>& tracker, const moving_object_msgs::MovingObjectPtr& observation) {
  // もともと位置だけを使っていたのを適当に位置＋速度の4次元に変更したが，あってるかは知らない
  // パラメータとかは調整するべき
  Eigen::Vector4d x;
  x[0] = observation->center.position.x;
  x[1] = observation->center.position.y;
  x[2] = observation->velocity.x;
  x[3] = observation->velocity.y;

  double sq_mahalanobis = tracker->squaredMahalanobisDistance(x);  

  // gating
  if(sq_mahalanobis > pow(3.0, 2) || (tracker->mean() - x).norm() > 1.5) {
    return boost::none;
  }
  return -kkl::math::gaussianProbMul(tracker->mean(), tracker->cov(), x);
}
  }
}

MovingObjectsTracker::MovingObjectsTracker()
{
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  data_association.reset(new kkl::alg::NearestNeighborAssociation<KalmanTracker::Ptr, moving_object_msgs::MovingObjectPtr>());

  id_gen_ = 0;

  private_node_handle_.reset(new ros::NodeHandle("~"));
  private_node_handle_->param("odom_frame", odom_frame_id_, std::string("odom"));

  reconfigure_function_ = boost::bind(&MovingObjectsTracker::reconfigureCallback, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_function_);
  
  moving_objects_sub_ = node_handle_.subscribe("moving_objects", 1, &MovingObjectsTracker::movingObjectsCallback, this);
  tracked_moving_objects_pub_ = private_node_handle_->advertise<moving_object_msgs::MovingObjectArray>("tracked_moving_objects", 1);
  trackers_covariance_pub_ = private_node_handle_->advertise<moving_object_tracker::TrackerCovarianceArray>("trackers_covariance", 1);
}

void MovingObjectsTracker::movingObjectsCallback(const moving_object_msgs::MovingObjectArrayConstPtr& moving_objects)
{
  geometry_msgs::TransformStamped to_odom;
  try {
    to_odom = tf_buffer_.lookupTransform(odom_frame_id_, moving_objects->header.frame_id, moving_objects->header.stamp);
  } 
  catch (tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("TF Exception: " << e.what());
    return;
  }

  std::vector<moving_object_msgs::MovingObjectPtr> transformed;
  transformed.reserve(moving_objects->moving_object_array.size());
  for (auto& moving_object : moving_objects->moving_object_array)
  {
    moving_object_msgs::MovingObjectPtr transformed_object(new moving_object_msgs::MovingObject);
    tf2::doTransform(moving_object.center, transformed_object->center, to_odom);
    tf2::doTransform(moving_object.velocity, transformed_object->velocity, to_odom);
    transformed_object->bounding_box = moving_object.bounding_box;
    transformed.push_back(transformed_object);
  }

  predict(moving_objects->header.stamp);
  correct(moving_objects->header.stamp, transformed);

  // publish tracked objects
  if(tracked_moving_objects_pub_.getNumSubscribers()) {
    moving_object_msgs::MovingObjectArray msg;
    msg.header.frame_id = odom_frame_id_;
    msg.header.stamp = moving_objects->header.stamp;
    msg.moving_object_array.reserve(trackers.size());
    for (auto &tracker : trackers)
    {
      if (tracker->correction_count() < correction_count_limit_)
        continue;

      if (tracker->lastCorrectionTime() != moving_objects->header.stamp)
        continue;

      moving_object_msgs::MovingObject obj_msg = *(boost::any_cast<moving_object_msgs::MovingObjectPtr>(tracker->lastAssociated()));
      obj_msg.id = tracker->id();
      obj_msg.center.position.x = tracker->position().x();
      obj_msg.center.position.y = tracker->position().y();
      obj_msg.velocity.x = tracker->velocity().x();
      obj_msg.velocity.y = tracker->velocity().y();
      msg.moving_object_array.push_back(obj_msg);
    }
    tracked_moving_objects_pub_.publish(msg);
  }

  if (trackers_covariance_pub_.getNumSubscribers())
  {
    moving_object_tracker::TrackerCovarianceArray msg;
    msg.header.frame_id = odom_frame_id_;
    msg.header.stamp = moving_objects->header.stamp;
    msg.tracker_covariance_array.reserve(trackers.size());

    for (auto &tracker : trackers)
    {
      // Remove intermittent objects
      if (tracker->correction_count() < correction_count_limit_)
        continue;

      if (tracker->lastCorrectionTime() != moving_objects->header.stamp)
        continue;

      if (tracker->lastAssociated().type() != typeid(moving_object_msgs::MovingObjectPtr))
        ROS_INFO("Type mismatch: now: %s, require: %s, any: %s", tracker->lastAssociated().type().name(), typeid(moving_object_msgs::MovingObjectPtr).name(), boost::any().type().name());
      else
      {
        moving_object_tracker::TrackerCovariance cov_msg;
        cov_msg.id = tracker->id();
        for (int i = 0; i < tracker->cov().size(); i++)
          cov_msg.covariance[i] = tracker->cov()(i);
        msg.tracker_covariance_array.push_back(cov_msg);
      }
    }
    trackers_covariance_pub_.publish(msg);
  }
}

void MovingObjectsTracker::predict(const ros::Time& time)
{
  for (auto& tracker : trackers)
    tracker->predict(time);
}

void MovingObjectsTracker::correct(const ros::Time& time, const std::vector<moving_object_msgs::MovingObjectPtr> &moving_objects)
{
  std::vector<bool> associated(moving_objects.size(), false);
  auto associations = data_association->associate(trackers, moving_objects);
  for(const auto& assoc : associations) {
    associated[assoc.observation] = true;
    auto &moving_object = moving_objects[assoc.observation];
    Eigen::Vector2d pos, vel;
    pos.x() = moving_object->center.position.x;
    pos.y() = moving_object->center.position.y;
    vel.x() = moving_object->velocity.x;
    vel.y() = moving_object->velocity.y;
    trackers[assoc.tracker]->correct(time, pos, vel, moving_object);
  }

  // generate new tracks
  for(int i=0; i<moving_objects.size(); i++) {
    if(!associated[i]) {
      // check if the detection is far from existing tracks
      bool close_to_tracker = false;
      for(const auto& tracker : trackers) {
        Eigen::Vector2d pos;
        pos.x() = moving_objects[i]->center.position.x;
        pos.y() = moving_objects[i]->center.position.y;
        if((tracker->position() - pos).norm() < object_radius_ * 2.0) { // TODO: Use object's bounding box
          close_to_tracker = true;
          break;
        }
      }

      if(close_to_tracker) {
        continue;
      }

      Eigen::Vector2d pos, vel;
      pos.x() = moving_objects[i]->center.position.x;
      pos.y() = moving_objects[i]->center.position.y;
      vel.x() = moving_objects[i]->velocity.x;
      vel.y() = moving_objects[i]->velocity.y;
      // generate a new track
      KalmanTracker::Ptr tracker(new KalmanTracker(id_gen_++, time, pos, vel, moving_objects[i]));
      trackers.push_back(tracker);
    }
  }

  // remove tracks with large covariance
  // 除去する分散の値を変えるべき
  auto remove_loc = std::partition(trackers.begin(), trackers.end(), [&](const KalmanTracker::Ptr& tracker) {
    return tracker->positionCov().trace() < covariance_trace_limit_;
  });
  trackers.erase(remove_loc, trackers.end());
  remove_loc = std::partition(trackers.begin(), trackers.end(), [&](const KalmanTracker::Ptr& tracker) {
    return tracker->velocityCov().trace() < covariance_trace_limit_;
  });
  trackers.erase(remove_loc, trackers.end());
}

void MovingObjectsTracker::reconfigureCallback(moving_object_tracker::MovingObjectsTrackerConfig &config, uint32_t level) {
  covariance_trace_limit_ = config.covariance_trace_limit;
  correction_count_limit_ = config.correction_count_limit;
  object_radius_ = config.object_radius;
}
