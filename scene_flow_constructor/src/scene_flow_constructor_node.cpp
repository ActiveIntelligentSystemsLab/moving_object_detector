#include "scene_flow_constructor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scene_flow_constructor");
  scene_flow_constructor::SceneFlowConstructor scene_flow_constructor;
  ros::spin();

  return 0;
}
