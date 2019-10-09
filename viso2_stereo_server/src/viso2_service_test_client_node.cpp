#include "viso2_service_test_client.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viso2_service_test_client");

  viso2_stereo_server::Viso2ServiceTestClient service_client;

  ros::spin();

  return 0;
}
