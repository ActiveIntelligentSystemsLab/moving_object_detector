#include "clusterer.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "clusterer");
  
  Clusterer clusterer;
  ros::spin();
  
  return 0;
}
