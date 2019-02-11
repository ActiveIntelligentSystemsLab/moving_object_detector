#include "moving_objects_tracker.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <kkl/alg/nearest_neighbor_association.hpp>

MovingObjectsTracker::MovingObjectsTracker()
{
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  moving_objects_sub_ = node_handle_.subscribe("moving_objects", 1, &MovingObjectsTracker::movingObjectsCallback, this);
  tracked_moving_objects_pub_ = node_handle_.advertise<moving_object_detector::MovingObjectArray>("tracked_moving_objects", 1);

  data_association.reset(new kkl::alg::NearestNeighborAssociation<KalmanTracker::Ptr, moving_object_detector::MovingObjectPtr>());
}

void MovingObjectsTracker::movingObjectsCallback(const moving_object_detector::MovingObjectArrayConstPtr& moving_objects)
{
  geometry_msgs::TransformStamped to_odom;
  to_odom = tf_buffer_.lookupTransform("odom", moving_objects->header.frame_id, moving_objects->header.stamp);

  moving_object_detector::MovingObjectArray transformed;
  transformed.moving_object_array.reserve(moving_objects->moving_object_array.size());
  for (auto& moving_object : moving_objects->moving_object_array)
  {
    moving_object_detector::MovingObject transformed_object;
    tf2::doTransform(moving_object.center, transformed_object.center, to_odom);
    tf2::doTransform(moving_object.velocity, transformed_object.velocity, to_odom);
    // bounding boxの座標変換も後でどうにかする
    transformed_object.bounding_box = moving_object.bounding_box;
    transformed.moving_object_array.push_back(transformed_object);
  }
  transformed.header.frame_id = "odom";
  transformed.header.stamp = moving_objects->header.stamp;

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
      msg.moving_object_array.push_back(boost::any_cast<moving_object_detector::MovingObject>(object->last_associated)); // コード汚すぎない？
    }
    tracked_moving_objects_pub_.publish(msg);
  }
}

void MovingObjectsTracker::predict(const ros::Time& time)
{
  for (auto& tracked_object : trackers)
    tracked_object->predict(time);
}

void MovingObjectsTracker::correct(const ros::Time& time, const moving_object_detector::MovingObjectArray &moving_objects)
{
  std::vector<bool> associated(moving_objects.moving_object_array.size(), false);
  auto associations = data_association->associate(trackers, moving_objects.moving_object_array);
  for(const auto& assoc : associations) {
    associated[assoc.observation] = true;
    auto &moving_object = moving_objects.moving_object_array[assoc.observation];
    Eigen::Vector2d pos, vel;
    pos.x = moving_object.center.x;
    pos.y = moving_object.center.y;
    vel.x = moving_object.velocity.x;
    vel.y = moving_object.velocity.y;
    trackers[assoc.tracker]->correct(time, pos, vel, moving_object);
  }

  // generate new tracks
  for(int i=0; i<moving_objects.moving_object_array.size(); i++) {
    if(!associated[i]) {
      // check if the detection is far from existing tracks
      bool close_to_tracker = false;
      for(const auto& object : moving_objects.moving_object_array) {
        Eigen::Vector2d pos;
        pos.x = moving_objects.moving_object_array[i].center.x;
        pos.y = moving_objects.moving_object_array[i].center.y;
        if((object->position() - pos).norm() < human_radius * 2.0) { // human_radiusは後で適当な変数名にかえるべき
          close_to_tracker = true;
          break;
        }
      }

      if(close_to_tracker) {
        continue;
      }

      Eigen::Vector2d pos, vel;
      pos.x = moving_objects.moving_object_array[i].center.x;
      pos.y = moving_objects.moving_object_array[i].center.y;
      vel.x = moving_objects.moving_object_array[i].velocity.x;
      vel.y = moving_objects.moving_object_array[i].velocity.y;
      // generate a new track
      KalmanTracker::Ptr tracker(new KalmanTracker(id_gen++, time, pos, vel, moving_objects.moving_object_array[i]));
      trackers.push_back(tracker);
    }
  }

  // remove tracks with large covariance
  // ここに速度分散による除去もつける
  auto remove_loc = std::partition(trackers.begin(), trackers.end(), [&](const KalmanTracker::Ptr& tracker) {
    return tracker->positionCov().trace() < remove_trace_thresh;
  });
  removed_objects.clear();
  std::copy(remove_loc, trackers.end(), std::back_inserter(removed_objects));
  trackers.erase(remove_loc, trackers.end());
}