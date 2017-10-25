#ifndef __HEADER_FLOW_3D__
#define __HEADER_FLOW_3D__

#include <tf2/LinearMath/Vector3.h>

class Flow3D {
public:
  tf2::Vector3 start;
  tf2::Vector3 end;
  
  Flow3D(tf2::Vector3 start, tf2::Vector3 end) : start(start), end(end) {}
  double length();
  tf2::Vector3 distanceVector();
  double radian2otherFlow(Flow3D& other_flow);
};

#endif