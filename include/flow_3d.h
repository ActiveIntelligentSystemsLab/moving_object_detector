#ifndef __HEADER_FLOW_3D__
#define __HEADER_FLOW_3D__

#include <tf2/LinearMath/Vector3.h>
#include <opencv2/core.hpp>

class Flow3D {
public:
  tf2::Vector3 start;
  tf2::Vector3 end;
  
  cv::Point2d start_uv;
  cv::Point2d end_uv;
  
  Flow3D(tf2::Vector3 start, tf2::Vector3 end, cv::Point2d start_uv, cv::Point2d end_uv) : start(start), end(end), start_uv(start_uv), end_uv(end_uv){}
  double length();
  tf2::Vector3 distanceVector();
  double radian2otherFlow(Flow3D& other_flow);
};

#endif
