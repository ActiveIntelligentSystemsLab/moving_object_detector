#include <ros/ros.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

namespace image_crop
{
  class ImageCrop
  {
    private:
      ros::NodeHandle nh_;
      std::shared_ptr<ros::NodeHandle> private_nh_;

      std::shared_ptr<image_transport::ImageTransport> it_;
      std::shared_ptr<image_transport::ImageTransport> private_it_;

      image_transport::CameraPublisher camera_pub_;
      image_transport::CameraSubscriber camera_sub_;

      int target_width_;
      int target_height_;
    public:
      ImageCrop();
      void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
  };
}
