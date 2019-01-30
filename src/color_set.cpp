#include "color_set.h"

ColorSet::ColorSet(int set_size)
{
  resize(set_size);
}

void ColorSet::getColor(int index, int &r, int &g, int &b)
{
  cv::Vec3b bgr = color_set_.at<cv::Vec3b>(index);
  b = bgr[0];
  g = bgr[1];
  r = bgr[2];
}

void ColorSet::resize(int set_size)
{
  if (set_size < 1)
    return;
  
  cv::Mat input_map(1, set_size, CV_8UC1);
  for (int i = 0; i < set_size; i++)
    input_map.at<uchar>(i) = i * 255 / set_size;
  
  cv::applyColorMap(input_map, color_set_, cv::ColormapTypes::COLORMAP_HSV);
}

size_t ColorSet::size()
{
  return color_set_.cols;
}