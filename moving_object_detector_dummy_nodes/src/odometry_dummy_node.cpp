#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher odometry_pub;

void dataCB(const sensor_msgs::ImageConstPtr& image)
{
  geometry_msgs::TransformStamped pub_msg;
  pub_msg.header = image->header;
  pub_msg.transform.rotation.x = 0.0;
  pub_msg.transform.rotation.y = 0.0;
  pub_msg.transform.rotation.z = 0.0;
  pub_msg.transform.rotation.w = 1.0;
  pub_msg.transform.translation.x = 0.0;
  pub_msg.transform.translation.y = 0.0;
  pub_msg.transform.translation.z = 0.0;
    
  odometry_pub.publish(pub_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_dummy_node");
  ros::NodeHandle nh;
  
  std::string odometry_topic = ros::this_node::getName() + nh.resolveName("camera_frame_transform");
  odometry_pub = nh.advertise<geometry_msgs::TransformStamped>(odometry_topic, 1);
  
  image_transport::ImageTransport img_trans = image_transport::ImageTransport(nh);
  std::string input_image_topic = nh.resolveName("stereo") + "/left/" + nh.resolveName("image");
  image_transport::Subscriber image_sub;
  image_sub = img_trans.subscribe(input_image_topic, 2, dataCB);
  
  ros::spin();

  return 0;
}

