#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

ros::Publisher flow_pub;

void dataCB(const sensor_msgs::ImageConstPtr& image)
{
  cv::Mat dummy_flow(image->height, image->width, CV_32FC1, 0.0);
  
  cv_bridge::CvImage cv_img(image->header, sensor_msgs::image_encodings::TYPE_32FC1, dummy_flow);
  
  flow_pub.publish(cv_img.toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optical_flow_dummy_node");
  ros::NodeHandle nh;
  
  std::string flow_topic = ros::this_node::getName() + "/flows";
  flow_pub = nh.advertise<sensor_msgs::Image>(flow_topic, 1);
  
  image_transport::ImageTransport img_trans = image_transport::ImageTransport(nh);
  std::string input_image_topic = nh.resolveName("image"); // image_transport::SubscriberFilter は何故か名前解決してくれないので
  image_transport::Subscriber image_sub;
  image_sub = img_trans.subscribe(input_image_topic, 2, dataCB);
  
  ros::spin();

  return 0;
}
