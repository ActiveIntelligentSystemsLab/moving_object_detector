#include "moving_objects_tracker.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <kkl/alg/nearest_neighbor_association.hpp>

namespace kkl {
  namespace alg {

/**
 * @brief definition of the distance between tracker and observation for data association
 */
template<>
boost::optional<double> distance(const std::shared_ptr<KalmanTracker>& tracker, const moving_object_detector::MovingObjectPtr& observation) {
  // もともと位置だけを使っていたのを適当に位置＋速度の4次元に変更したが，あってるかは知らない
  // パラメータとかは調整するべき
  Eigen::Vector4d x;
  x[0] = observation->center.position.x;
  x[1] = observation->center.position.y;
  x[2] = observation->velocity.y;
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

  data_association.reset(new kkl::alg::NearestNeighborAssociation<KalmanTracker::Ptr, moving_object_detector::MovingObjectPtr>());

  object_radius_ = 0.5; // 後で調整する
  covariance_trace_limit_ = 0.5;
  id_gen_ = 0;

  moving_objects_sub_ = node_handle_.subscribe("moving_objects", 1, &MovingObjectsTracker::movingObjectsCallback, this);
  tracked_moving_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("tracked_moving_objects", 1);

}

void MovingObjectsTracker::movingObjectsCallback(const moving_object_detector::MovingObjectArrayConstPtr& moving_objects)
{
  geometry_msgs::TransformStamped to_odom;
  to_odom = tf_buffer_.lookupTransform("odom", moving_objects->header.frame_id, moving_objects->header.stamp);

  std::vector<moving_object_detector::MovingObjectPtr> transformed;
  transformed.reserve(moving_objects->moving_object_array.size());
  for (auto& moving_object : moving_objects->moving_object_array)
  {
    moving_object_detector::MovingObjectPtr transformed_object(new moving_object_detector::MovingObject);
    tf2::doTransform(moving_object.center, transformed_object->center, to_odom);
    tf2::doTransform(moving_object.velocity, transformed_object->velocity, to_odom);
    transformed_object->bounding_box = moving_object.bounding_box;
    transformed.push_back(transformed_object);
  }

  predict(moving_objects->header.stamp);
  correct(moving_objects->header.stamp, transformed);

  // publish tracks msg
  // moving_objectにID付けるのと，別トピックかなんかでcovarianceの情報とか出した方がいい
  if(tracked_moving_objects_pub_.getNumSubscribers()) {
    moving_object_detector::MovingObjectArray msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = moving_objects->header.stamp;
    msg.moving_object_array.reserve(trackers.size());
    for (auto &object : trackers)
    {
      if (object->correction_count() < 5)
        continue;

      moving_object_detector::MovingObject obj_msg = *(boost::any_cast<moving_object_detector::MovingObjectPtr>(object->lastAssociated()));
      obj_msg.id = object->id();
      obj_msg.center.position.x = object->position().x();
      obj_msg.center.position.y = object->position().y();
      obj_msg.velocity.x = object->velocity().x();
      obj_msg.velocity.y = object->velocity().y();
      msg.moving_object_array.push_back(obj_msg); // コード汚すぎない？
    }
    tracked_moving_objects_pub_.publish(msg);
  }
}

void MovingObjectsTracker::predict(const ros::Time& time)
{
  for (auto& tracked_object : trackers)
    tracked_object->predict(time);
}

void MovingObjectsTracker::correct(const ros::Time& time, const std::vector<moving_object_detector::MovingObjectPtr> &moving_objects)
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
      for(const auto& object : trackers) {
        Eigen::Vector2d pos;
        pos.x() = moving_objects[i]->center.position.x;
        pos.y() = moving_objects[i]->center.position.y;
        if((object->position() - pos).norm() < object_radius_ * 2.0) { 
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
  // ここに速度分散による除去もつける
  auto remove_loc = std::partition(trackers.begin(), trackers.end(), [&](const KalmanTracker::Ptr& tracker) {
    return tracker->positionCov().trace() < covariance_trace_limit_;
  });
  removed_objects.clear();
  std::copy(remove_loc, trackers.end(), std::back_inserter(removed_objects));
  trackers.erase(remove_loc, trackers.end());
}