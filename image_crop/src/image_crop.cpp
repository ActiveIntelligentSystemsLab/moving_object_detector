#include "image_crop.h"
#include <cv_bridge/cv_bridge.h>

namespace image_crop
{
  ImageCrop::ImageCrop()
  {
    private_nh_.reset(new ros::NodeHandle("~"));

    private_nh_->getParam("target_width", target_width_);
    private_nh_->getParam("target_height", target_height_);

    it_.reset(new image_transport::ImageTransport(nh_));
    private_it_.reset(new image_transport::ImageTransport(*private_nh_));

    camera_pub_ = private_it_->advertiseCamera("cropped_image", 1);
    camera_sub_ = it_->subscribeCamera("image", 1, &ImageCrop::cameraCallback, this);
  }

  void ImageCrop::cameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info)
  {
    // crop image
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(image_msg, "bgr8");
    cv::Rect roi((image_msg->width - target_width_) / 2, (image_msg->height - target_height_) / 2, target_width_, target_height_);
    cv::Mat roi_image = image->image(roi);
    cv_bridge::CvImage cropped_image;
    roi_image.copyTo(cropped_image.image);
    cropped_image.header = image_msg->header;
    cropped_image.encoding = "bgr8";

    // camera info
    double cropped_cx = camera_info->K[2] - (image_msg->width - target_width_) / 2;
    double cropped_cy = camera_info->K[5] - (image_msg->height - target_height_) / 2;
    sensor_msgs::CameraInfo cropped_caminfo = *camera_info;
    cropped_caminfo.K[2] = cropped_cx;
    cropped_caminfo.K[5] = cropped_cy;
    cropped_caminfo.P[2] = cropped_cx;
    cropped_caminfo.P[6] = cropped_cy;
    cropped_caminfo.width = target_width_;
    cropped_caminfo.height = target_height_;

    camera_pub_.publish(*(cropped_image.toImageMsg()), cropped_caminfo);
  }
}
