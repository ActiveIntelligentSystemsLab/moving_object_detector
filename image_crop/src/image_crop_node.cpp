#include "image_crop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_crop");

  image_crop::ImageCrop image_crop;
  ros::spin();

  return 0;
}
