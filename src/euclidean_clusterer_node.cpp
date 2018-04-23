#include "euclidean_clusterer.h"

int main(int argc, char ** argv) {
  ros::init(argc, argv, "clusterer");

  EuclideanClusterer clusterer;
  ros::spin();

  return 0;
}
