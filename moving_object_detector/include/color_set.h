#ifndef __HEADER_COLOR_SET__
#define __HEADER_COLOR_SET__

#include <opencv2/imgproc.hpp>

class ColorSet
{
public:
  ColorSet(int set_size = 10);
  void getColor(int index, int &r, int &g, int &b);
  void resize(int set_size);
  size_t size();
private:
  cv::Mat color_set_;
};

#endif