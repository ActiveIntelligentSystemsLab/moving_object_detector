#ifndef __HEADER_3D_RECONTRUCTION__
#define __HEADER_3D_RECONTRUCTION__

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Vector3.h>

bool getPoint3D(int u, int v, image_geometry::PinholeCameraModel& camera_model, const sensor_msgs::Image& depth_image, tf2::Vector3& point3d);

#endif
