#include <moving_objects_tracker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_object_tracker_node");
  MovingObjectsTracker tracker;
  ros::spin();
  return 0;
}