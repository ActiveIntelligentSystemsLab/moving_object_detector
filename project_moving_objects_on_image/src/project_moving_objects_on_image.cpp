#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_common.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <moving_object_msgs/MovingObjectArray.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

image_transport::Publisher image_pub;
int red, green, blue;

void dataCB(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info, const moving_object_msgs::MovingObjectArrayConstPtr& moving_object_array)
{
  cv::Mat cv_image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    cv_image = input_bridge->image;
  } catch(cv_bridge::Exception& exception) {
    ROS_ERROR("Failed to Convert image from msg to cv");
  }

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  for (auto& moving_object : moving_object_array->moving_object_array) {
    cv::Point3d top_left_pt(moving_object.center.position.x - (moving_object.bounding_box.x / 2), moving_object.center.position.y - (moving_object.bounding_box.y / 2), moving_object.center.position.z - (moving_object.bounding_box.z / 2));

    cv::Point2d top_left_uv = camera_model.project3dToPixel(top_left_pt);
    cv::Point3d bottom_right_pt(moving_object.center.position.x + (moving_object.bounding_box.x / 2), moving_object.center.position.y + (moving_object.bounding_box.y / 2), moving_object.center.position.z + (moving_object.bounding_box.z / 2));
    cv::Point2d bottom_right_uv = camera_model.project3dToPixel(bottom_right_pt);

    cv::rectangle(cv_image, top_left_uv, bottom_right_uv, CV_RGB(red, green, blue), 2);
  }

  image_pub.publish(input_bridge->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "project_moving_objects_on_image");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  red = private_nh.param("red", 255);
  green = private_nh.param("green", 0);
  blue = private_nh.param("blue", 0);

  image_transport::ImageTransport img_trans = image_transport::ImageTransport(nh);
  std::string input_image_topic = nh.resolveName("input_image"); // image_transport::SubscriberFilter は何故か名前解決してくれないので
  std::string input_camerainfo_topic = image_transport::getCameraInfoTopic(input_image_topic);

  image_pub = img_trans.advertise("moving_objects_image", 1);
  image_transport::SubscriberFilter image_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camerainfo_sub;
  message_filters::Subscriber<moving_object_msgs::MovingObjectArray> moving_objects_sub;

  image_sub.subscribe(img_trans, input_image_topic, 10);
  moving_objects_sub.subscribe(nh, "moving_objects", 10);
  camerainfo_sub.subscribe(nh, input_camerainfo_topic, 10);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, moving_object_msgs::MovingObjectArray> sync(image_sub, camerainfo_sub, moving_objects_sub, 10);
  sync.registerCallback(boost::bind(&dataCB, _1, _2, _3));

  ros::spin();

  return 0;
}
