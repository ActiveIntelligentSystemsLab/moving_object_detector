#include <moving_object_detector/MovingObjectArray.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher markers_pub;

void callback(const moving_object_detector::MovingObjectArrayConstPtr& moving_object_msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "display_moving_objects");
  ros::NodeHandle node_handle;
  
  markers_pub = node_handle.advertise<visualization_msgs::MarkerArray>("moving_object_markers", 10);
  ros::Subscriber moving_objects_sub = node_handle.subscribe("moving_objects", 10, callback);
  
  ros::spin();
}

void callback(const moving_object_detector::MovingObjectArrayConstPtr& moving_objects_msg) {
  visualization_msgs::MarkerArray pub_msg;

  // First element is used for delete markers in previous frame
  visualization_msgs::Marker delete_marker;
  delete_marker.header = moving_objects_msg->header;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  pub_msg.markers.push_back(delete_marker);
  
  int i = 0;
  for (auto& moving_object : moving_objects_msg->moving_object_array) {
    visualization_msgs::Marker marker;
    marker.header = moving_objects_msg->header;
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = moving_object.center;
    marker.scale.x = moving_object.bounding_box.x;
    marker.scale.y = moving_object.bounding_box.y;
    marker.scale.z = moving_object.bounding_box.z;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    pub_msg.markers.push_back(marker);
    i++;
  }
  
  markers_pub.publish(pub_msg);
}
