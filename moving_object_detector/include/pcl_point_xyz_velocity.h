#ifndef __PCL_POINT_XYZ_VELOCITY_HEADER__
#define __PCL_POINT_XYZ_VELOCITY_HEADER__

#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZVelocity
{
  PCL_ADD_POINT4D;
  
  union
  {
    float data_velocity[4];
    struct
    {
      float vx;
      float vy;
      float vz;
    };
  };
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZVelocity,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, vx, vx)
                                   (float, vy, vy)
                                   (float, vz, vz)
)

#endif