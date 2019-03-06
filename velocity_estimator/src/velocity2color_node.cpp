#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velocity_estimator/pcl_point_xyz_velocity.h>
#include <velocity_estimator/Velocity2ColorConfig.h>

#include <cmath>

ros::Publisher color_pub;
double max_velocity;

void callback(const sensor_msgs::PointCloud2ConstPtr pc_velocity);
void reconfigureCB(moving_object_detector::Velocity2ColorConfig& config, uint32_t level);

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity2color");
  ros::NodeHandle node_handle;
  
  dynamic_reconfigure::Server<moving_object_detector::Velocity2ColorConfig> reconfigure_server;
  dynamic_reconfigure::Server<moving_object_detector::Velocity2ColorConfig>::CallbackType reconfigure_func;
  
  reconfigure_func = boost::bind(&reconfigureCB, _1, _2);
  reconfigure_server.setCallback(reconfigure_func);
  
  color_pub = node_handle.advertise<sensor_msgs::PointCloud2>("color_pc", 10);
  ros::Subscriber velocity_sub = node_handle.subscribe("velocity_pc", 10, callback);
  
  ros::spin();
}

void callback(const sensor_msgs::PointCloud2ConstPtr velocity_pc_msg) {
  if (color_pub.getNumSubscribers() <= 0)
    return;

  pcl::PointCloud<pcl::PointXYZVelocity> velocity_pc;
  pcl::fromROSMsg(*velocity_pc_msg, velocity_pc);
  
  pcl::PointCloud<pcl::PointXYZRGB> color_pc;
  
  for (auto &velocity_point : velocity_pc) {
    uint32_t red, blue, green;
    
    red = std::abs(velocity_point.vx) / max_velocity * 255;
    if (red > 255)
      red = 255;
    
    green = std::abs(velocity_point.vy) / max_velocity * 255;
    if (green > 255)
      green = 255;
    
    blue = std::abs(velocity_point.vz) / max_velocity * 255;
    if (blue > 255)
      blue = 255;
    
    uint32_t rgb = red << 16 | green << 8 | blue;
    pcl::PointXYZRGB color_point;
    color_point.rgb = *reinterpret_cast<float*>(&rgb);
    
    color_point.x = velocity_point.x;
    color_point.y = velocity_point.y;
    color_point.z = velocity_point.z;
    
    color_pc.push_back(color_point);
  }
  
  sensor_msgs::PointCloud2 color_pc_msg;
  pcl::toROSMsg(color_pc, color_pc_msg);
  color_pc_msg.header = velocity_pc_msg->header;  
  color_pub.publish(color_pc_msg);
}

void reconfigureCB(moving_object_detector::Velocity2ColorConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request: velocity_max = %f", config.max_velocity);
  max_velocity = config.max_velocity;
}
