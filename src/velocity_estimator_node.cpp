#include "ros/ros.h"
#include "velocity_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_estimator");
  
  VelocityEstimator detector;
  
  ros::spin();
  
  return 0;
}
