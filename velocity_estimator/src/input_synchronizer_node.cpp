#include "ros/ros.h"
#include "input_synchronizer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "input_synchronizer");
  
  InputSynchronizer input_synchronizer;
  
  ros::spin();
  
  return 0;
}
