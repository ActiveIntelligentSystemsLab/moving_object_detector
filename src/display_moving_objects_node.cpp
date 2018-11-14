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
    visualization_msgs::Marker bounding_box;
    bounding_box.header = moving_objects_msg->header;
    bounding_box.id = i;
    bounding_box.type = visualization_msgs::Marker::CUBE;
    bounding_box.action = visualization_msgs::Marker::ADD;
    bounding_box.pose.position = moving_object.center;
    bounding_box.scale.x = moving_object.bounding_box.x;
    bounding_box.scale.y = moving_object.bounding_box.y;
    bounding_box.scale.z = moving_object.bounding_box.z;
    bounding_box.color.a = 0.5;
    bounding_box.color.r = 1.0;
    bounding_box.color.g = 0.0;
    bounding_box.color.b = 0.0;
    pub_msg.markers.push_back(bounding_box);
    i++;
    
    visualization_msgs::Marker velocity_arrow;
    velocity_arrow.header = moving_objects_msg->header;
    velocity_arrow.id = i;
    velocity_arrow.type = visualization_msgs::Marker::ARROW;
    velocity_arrow.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point arrow_start = moving_object.center;
    geometry_msgs::Point arrow_end;
    arrow_end.x = moving_object.center.x + moving_object.velocity.x;
    arrow_end.y = moving_object.center.y + moving_object.velocity.y;
    arrow_end.z = moving_object.center.z + moving_object.velocity.z;
    velocity_arrow.points.push_back(arrow_start);
    velocity_arrow.points.push_back(arrow_end);
    double shaft_diameter = 0.1;
    double head_diameter = 0.2;
    double head_length = 0.0; // Use default value
    velocity_arrow.scale.x = shaft_diameter;
    velocity_arrow.scale.y = head_diameter;
    velocity_arrow.scale.z = head_length;
    velocity_arrow.color.a = 1.0;
    velocity_arrow.color.r = 1.0;
    velocity_arrow.color.g = 0.0;
    velocity_arrow.color.b = 0.0;
    pub_msg.markers.push_back(velocity_arrow);
    i++;
  }
  
  markers_pub.publish(pub_msg);
}
