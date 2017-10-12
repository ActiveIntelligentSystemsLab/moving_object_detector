#include "ros/ros.h"
#include "moving_object_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_object_detector");
  
  MovingObjectDetector detector;
  
  ros::spin();
  
  return 0;
}