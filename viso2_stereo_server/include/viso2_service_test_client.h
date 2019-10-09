#ifndef VISO2_STEREO_SERVER_H_
#define VISO2_STEREO_SERVER_H_

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <memory>

namespace viso2_stereo_server 
{
  class Viso2ServiceTestClient 
  {
  private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    // Subscribers for stereo image and camera info
    image_transport::SubscriberFilter left_image_subscriber_;
    image_transport::SubscriberFilter right_image_subscriber_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_subscriber_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_subscriber_;

    /**
     * \brief Type alias of TimeSynchrozier for stereo image and camera info
     */
    using StereoSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>;
    /**
     * \brief For synchronize timestamp of stereo image and camera info
     */
    std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;

    /**
     * \brief Client of EstimateMotionFromStereo service
     */
    ros::ServiceClient estimate_motion_client_;

    /**
     * \brief Callback function of stereo_synchronizer_
     */
    void stereoSynchronizerCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);

  public:
    Viso2ServiceTestClient();
  };
}

#endif
