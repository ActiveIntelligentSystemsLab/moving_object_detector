#include "flow_3d.h"
#include "moving_object_detector.h"
#include "pcl_point_xyz_velocity.h"
#include "process_disparity_image.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_common.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

MovingObjectDetector::MovingObjectDetector() {  
  first_run_ = true;
  
  reconfigure_func_ = boost::bind(&MovingObjectDetector::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  pc_with_velocity_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pc_with_velocity", 10);
  cluster_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("cluster", 10);
  removed_by_matching_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("removed_by_matching", 10);
  
  camera_transform_sub_.subscribe(node_handle_, "camera_transform", 1);
  optical_flow_left_sub_.subscribe(node_handle_, "optical_flow_left", 1); // optical flowはrectified imageで計算すること
  optical_flow_right_sub_.subscribe(node_handle_, "optical_flow_right", 1); // optical flowはrectified imageで計算すること
  disparity_image_sub_.subscribe(node_handle_, "disparity_image", 1);
  std::string left_camera_topic = node_handle_.resolveName("synchronizer_output_left_rect_image");
  std::string left_camera_info_topic = image_transport::getCameraInfoTopic(left_camera_topic);
  left_camera_info_sub_.subscribe(node_handle_, left_camera_info_topic, 1);

  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage>>(camera_transform_sub_, optical_flow_left_sub_, optical_flow_right_sub_, left_camera_info_sub_, disparity_image_sub_, 1);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::dataCB, this, _1, _2, _3, _4, _5));
  
  input_synchronizer_ = std::make_shared<InputSynchronizer>(*this);
}

void MovingObjectDetector::reconfigureCB(moving_object_detector::MovingObjectDetectorConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: downsample_scale = %d, moving_flow_length = %f, flow_length_diff = %f, flow_start_diff = %f, flow_radian_diff = %f, flow_axis_max = %f, matching_tolerance = %f, cluster_element_num = %d", config.downsample_scale, config.moving_flow_length, config.flow_length_diff, config.flow_start_diff, config.flow_radian_diff, config.flow_axis_max, config.matching_tolerance, config.cluster_element_num);
  
  downsample_scale_ = config.downsample_scale;
  moving_flow_length_ = config.moving_flow_length;
  flow_length_diff_ = config.flow_length_diff;
  flow_start_diff_ = config.flow_start_diff;
  flow_radian_diff_ = config.flow_radian_diff;
  flow_axis_max_ = config.flow_axis_max;
  matching_tolerance_ = config.matching_tolerance;
  cluster_element_num_ = config.cluster_element_num;
}

void MovingObjectDetector::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const sensor_msgs::ImageConstPtr& optical_flow_left, const sensor_msgs::ImageConstPtr& optical_flow_right, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  // 次の入力データをVISO2とflowノードに送信し，移動物体検出を行っている間に処理させる
  input_synchronizer_->publish();
  
  ros::Time start_process = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZVelocity> pc_with_velocity;
  pcl::PointCloud<pcl::PointXYZ> removed_by_matching;
    
  if (first_run_) {
    first_run_ = false;
  } else {
    std::list<std::list<Flow3D>> clusters; // clusters[cluster_index][cluster_element_index]
    
    // flow_mapの(x, y)には(dx,dy)が格納されている
    // つまり，フレームtの注目点の座標を(x,y)とすると，それに対応するフレームt-1の座標は(x-dx,y-dy)となる
    cv::Mat flow_map_left = cv_bridge::toCvShare(optical_flow_left)->image;
    cv::Mat flow_map_right = cv_bridge::toCvShare(optical_flow_right)->image;
    
    ProcessDisparityImage disparity_processor(disparity_image, left_camera_info);
    ProcessDisparityImage disparity_processor_previous(disparity_image_previous_, left_camera_info);

    ros::Duration time_between_frames = camera_transform->header.stamp - time_stamp_previous_;
    
    for (int left_now_y = 0; left_now_y < flow_map_left.rows; left_now_y += downsample_scale_) 
    {
      for (int left_now_x = 0; left_now_x < flow_map_left.cols; left_now_x += downsample_scale_)
      {
        cv::Point2i left_previous, left_now, right_now, right_previous;
        left_now.x = left_now_x;
        left_now.y = left_now_y;
        
        cv::Vec2f flow_left = flow_map_left.at<cv::Vec2f>(left_now.y, left_now.x);
        
        if(std::isnan(flow_left[0]) || std::isnan(flow_left[1]))
          continue;
        
        left_previous.x = std::round(left_now.x - flow_left[0]);
        left_previous.y = std::round(left_now.y - flow_left[1]);
        
        tf2::Vector3 point3d_now, point3d_previous;
        if(!disparity_processor.getPoint3D(left_now.x, left_now.y, point3d_now))
          continue;
        if(!disparity_processor_previous.getPoint3D(left_previous.x, left_previous.y, point3d_previous))
          continue;
        
        float disparity_now;
        if (!disparity_processor.getDisparity(left_now.x, left_now.y, disparity_now))
          continue;
        if (std::isnan(disparity_now) || std::isinf(disparity_now) || disparity_now < 0)
          continue;
        
        float disparity_previous;
        if (!disparity_processor_previous.getDisparity(left_previous.x, left_previous.y, disparity_previous))
          continue;
        if (std::isnan(disparity_previous) || std::isinf(disparity_previous) || disparity_previous < 0)
          continue;
        
        right_now.x = std::round(left_now.x - disparity_now);
        right_now.y = left_now.y;
        
        right_previous.x = std::round(left_previous.x - disparity_previous);
        right_previous.y = left_previous.y;
        
        if (right_now.x < 0 || right_now.x >= flow_map_right.cols)
          continue;
        
        cv::Vec2f flow_right = flow_map_right.at<cv::Vec2f>(right_now.y, right_now.x);
        
        if(std::isnan(flow_right[0]) || std::isnan(flow_right[1]))
          continue;

        if (matching_tolerance_ >= 0) { // matching_toleranceが負なら無効化
          double x_diff = right_previous.x + flow_right[0] - right_now.x;
          double y_diff = right_previous.y + flow_right[1] - right_now.y;
          double diff = std::sqrt(x_diff * x_diff + y_diff * y_diff);
          if (diff > matching_tolerance_) {
            if (removed_by_matching_pub_.getNumSubscribers() > 0) 
            {
              pcl::PointXYZ pcl_point;
              pcl_point.x = point3d_now.getX();
              pcl_point.y = point3d_now.getY();
              pcl_point.z = point3d_now.getZ();
              removed_by_matching.push_back(pcl_point);
            }
            
            continue;
          }
        }
        
        // 以前のフレームを現在のフレームに座標変換
        // VISO2からメッセージとして取得するtfは，現在の点を前フレームの座標系に座標変換する行列である
        tf2::Stamped<tf2::Transform> tf_now2previous;
        tf2::fromMsg(*camera_transform, tf_now2previous);
        tf2::Vector3 point3d_previous_transformed = tf_now2previous.inverse() * point3d_previous;
        
        Flow3D flow3d = Flow3D(point3d_previous_transformed, point3d_now, left_previous, left_now);
        
        pcl::PointXYZVelocity point_with_velocity;
        point_with_velocity.x = flow3d.end.getX();
        point_with_velocity.y = flow3d.end.getY();
        point_with_velocity.z = flow3d.end.getZ();
        
        tf2::Vector3 flow3d_vector = flow3d.distanceVector();
        point_with_velocity.vx = flow3d_vector.getX() / time_between_frames.toSec();
        point_with_velocity.vy = flow3d_vector.getY() / time_between_frames.toSec();
        point_with_velocity.vz = flow3d_vector.getZ() / time_between_frames.toSec();
        
        pc_with_velocity.push_back(point_with_velocity);
        
        if (cluster_pub_.getNumSubscribers() > 0)
        {
          
          if (flow3d.length() / time_between_frames.toSec() < moving_flow_length_ )
            continue;
          
          bool already_clustered = false;
          std::list<std::list<Flow3D>>::iterator belonged_cluster_it;
          for (auto cluster_it = clusters.begin(); cluster_it != clusters.end(); cluster_it++) {
            for (auto& clustered_flow : *cluster_it) {
              if (flow_start_diff_ < (flow3d.start - clustered_flow.start).length())
                continue;
              if (flow_length_diff_ < std::abs(flow3d.length() - clustered_flow.length()))
                continue;
              if (flow_radian_diff_ < flow3d.radian2otherFlow(clustered_flow))
                continue;
              
              if (!already_clustered) {
                cluster_it->push_back(flow3d);
                already_clustered = true;
                belonged_cluster_it = cluster_it;
              } else {
                // クラスタに所属済みの点が，他のクラスタにも属していればクラスタ同士を結合する
                auto tmp_cluster_it = cluster_it;
                cluster_it--; // 現在のクラスタが消去されるので，イテレータを一つ前に戻す 次のループのインクリメントで消去されたクラスタの次のクラスタに到達することになる
                belonged_cluster_it->splice(belonged_cluster_it->end(), *tmp_cluster_it);
                clusters.erase(tmp_cluster_it);
              }
              
              break;
            }
          }
          
          // どのクラスタにも振り分けられなければ，新しいクラスタを作成
          if (!already_clustered) {
            clusters.emplace_back();
            clusters.back().push_back(flow3d);
          }
        }
      }
    }
    
    sensor_msgs::PointCloud2 pc_with_velocity_msg;
    pcl::toROSMsg(pc_with_velocity, pc_with_velocity_msg);
    pc_with_velocity_msg.header.frame_id = left_camera_info->header.frame_id;
    pc_with_velocity_msg.header.stamp = left_camera_info->header.stamp;
    pc_with_velocity_pub_.publish(pc_with_velocity_msg);
    
    if (removed_by_matching_pub_.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(removed_by_matching, pointcloud_msg);
      pointcloud_msg.header.frame_id = left_camera_info->header.frame_id;
      pointcloud_msg.header.stamp = left_camera_info->header.stamp;
      removed_by_matching_pub_.publish(pointcloud_msg);
    }
    
    if (cluster_pub_.getNumSubscribers() > 0)
    {
      // 要素数の少なすぎるクラスタの削除
      for (auto cluster_it = clusters.begin(); cluster_it != clusters.end(); cluster_it++) {
        if (cluster_it->size() < cluster_element_num_) {
          auto tmp_cluster_it = cluster_it;
          cluster_it--; // 現在のクラスタを消去するので，1つ前のクラスタに戻ることで，次のループのインクリメントで消去したクラスタの次にクラスタに移動できる
          clusters.erase(tmp_cluster_it);
        }
      }
      
      pcl::PointCloud<pcl::PointXYZI> cluster_pcl;
      int i = 0;
      for (auto cluster_it = clusters.begin(); cluster_it != clusters.end(); cluster_it++) {
        for (auto& clustered_flow : *cluster_it) {
          pcl::PointXYZI point_clustered;
          
          point_clustered.x = clustered_flow.end.getX();
          point_clustered.y = clustered_flow.end.getY();
          point_clustered.z = clustered_flow.end.getZ();
          point_clustered.intensity = i;
          
          cluster_pcl.push_back(point_clustered);
        }
        i++;
      }
      
      sensor_msgs::PointCloud2 cluster_msg;
      pcl::toROSMsg(cluster_pcl, cluster_msg);
      cluster_msg.header.frame_id = left_camera_info->header.frame_id;
      cluster_msg.header.stamp = left_camera_info->header.stamp;
      cluster_pub_.publish(cluster_msg);
    }
  }
  disparity_image_previous_ = disparity_image;
  time_stamp_previous_ = camera_transform->header.stamp;
  
  ros::Duration process_time = ros::Time::now() - start_process;
  ROS_INFO("process time: %f", process_time.toSec());
}

MovingObjectDetector::InputSynchronizer::InputSynchronizer(MovingObjectDetector& outer_instance)
{
  image_transport_ = std::make_shared<image_transport::ImageTransport>(outer_instance.node_handle_);
  
  std::string publish_left_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_output_left_rect_image");
  std::string publish_right_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_output_right_rect_image");
  
  left_rect_image_pub_ = image_transport_->advertiseCamera(publish_left_rect_image_topic, 1);
  right_rect_image_pub_ = image_transport_->advertiseCamera(publish_right_rect_image_topic, 1);
  
  std::string subscribe_left_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_input_left_rect_image");
  std::string subscribe_right_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_input_right_rect_image");
  
  left_rect_image_sub_.subscribe(*image_transport_, subscribe_left_rect_image_topic, 10);
  left_rect_info_sub_.subscribe(outer_instance.node_handle_, image_transport::getCameraInfoTopic(subscribe_left_rect_image_topic), 10);
  right_rect_image_sub_.subscribe(*image_transport_, subscribe_right_rect_image_topic, 10);
  right_rect_info_sub_.subscribe(outer_instance.node_handle_, image_transport::getCameraInfoTopic(subscribe_right_rect_image_topic), 10);
  time_sync_ = std::make_shared<DataTimeSynchronizer>(left_rect_image_sub_, left_rect_info_sub_, right_rect_image_sub_, right_rect_info_sub_, 10);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::InputSynchronizer::dataCallBack, this, _1, _2, _3, _4));
}

void MovingObjectDetector::InputSynchronizer::dataCallBack(const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info)
{
  static int count = 0;
  
  left_rect_image_ = *left_rect_image;
  left_rect_info_ = *left_rect_info;
  right_rect_image_ = *right_rect_image;
  right_rect_info_ = *right_rect_info;
  
  // MovingObjectDetector内でinput_synchronizer->publish()を行わない限り処理は開始しないので，初期2フレーム分のデータを送信する
  if (count < 2) {
    publish();
    count++;
  }
  else if ((ros::Time::now() - last_publish_time_).toSec() > 0.5) { // publishが途中で停止した場合には復帰
    publish();
  }
}

void MovingObjectDetector::InputSynchronizer::publish()
{
  left_rect_image_pub_.publish(left_rect_image_, left_rect_info_);
  right_rect_image_pub_.publish(right_rect_image_, right_rect_info_);
  
  last_publish_time_ = ros::Time::now();
}
