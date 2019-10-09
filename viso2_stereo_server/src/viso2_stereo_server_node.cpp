#include "viso2_stereo_server.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "viso2_stereo_server");

  viso2_stereo_server::Viso2StereoServer server;

  ros::spin();
  return 0;
}
