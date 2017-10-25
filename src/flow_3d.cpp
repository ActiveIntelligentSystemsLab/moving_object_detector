#include "flow_3d.h"

#include <cmath>

double Flow3D::length()
{
  return (end - start).length();
}

tf2::Vector3 Flow3D::distanceVector()
{
  return end - start;
}

double Flow3D::radian2otherFlow(Flow3D& other_flow)
{
  return distanceVector().angle(other_flow.distanceVector());
}
