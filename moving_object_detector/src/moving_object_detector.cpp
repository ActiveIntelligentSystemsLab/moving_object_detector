#include "moving_object_detector.h"
#include "flow_3d.h"
#include "moving_object_detector/MatchPointArray.h"
#include "3d_recontruction.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/camera_common.h>
#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>

MovingObjectDetector::MovingObjectDetector() {  
  first_run_ = true;
  
  reconfigure_func_ = boost::bind(&MovingObjectDetector::reconfigureCB, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_func_);
  
  ros::param::param("~downsample_scale", downsample_scale_, 10);
  ros::param::param("~confidence_limit", confidence_limit_, 95);
  ros::param::param("~moving_flow_length", moving_flow_length_, 0.10);
  ros::param::param("~flow_length_diff", flow_length_diff_, 0.10);
  ros::param::param("~flow_start_diff", flow_start_diff_, 0.10);
  ros::param::param("~flow_radian_diff", flow_radian_diff_, 0.17);
  ros::param::param("~flow_axis_max_", flow_axis_max_, 0.5);
  ros::param::param("~matching_tolerance_", matching_tolerance_, 2.0);
  ros::param::param("~cluster_element_num", cluster_element_num_, 10);
  
  flow3d_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("flow3d", 10);
  cluster_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("cluster", 10);
  removed_by_matching_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("removed_by_matching", 10);
  removed_by_confidence_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("removed_by_confidence", 1);
  
  std::string depth_image_topic = node_handle_.resolveName("depth_image_rectified"); // image_transport::SubscriberFilter は何故か名前解決してくれないので
  std::string depth_image_info_topic = image_transport::getCameraInfoTopic(depth_image_topic);
  
  camera_transform_sub_.subscribe(node_handle_, "camera_transform", 2);
  optical_flow_left_sub_.subscribe(node_handle_, "optical_flow_left", 2); // optical flowはrectified imageで計算すること
  optical_flow_right_sub_.subscribe(node_handle_, "optical_flow_right", 2); // optical flowはrectified imageで計算すること
  disparity_image_sub_.subscribe(node_handle_, "disparity_image", 2);
  depth_image_sub_.subscribe(node_handle_, depth_image_topic, 2);
  depth_image_info_sub_.subscribe(node_handle_, depth_image_info_topic, 2);
  confidence_map_sub_.subscribe(node_handle_, "confidence_map", 2);

  time_sync_ = std::make_shared<message_filters::TimeSynchronizer<geometry_msgs::TransformStamped, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, stereo_msgs::DisparityImage>>(camera_transform_sub_, optical_flow_left_sub_, optical_flow_right_sub_, depth_image_sub_, depth_image_info_sub_, confidence_map_sub_, disparity_image_sub_, 2);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::dataCB, this, _1, _2, _3, _4, _5, _6, _7));
  
  input_synchronizer_ = std::make_shared<InputSynchronizer>(*this);
}

void MovingObjectDetector::reconfigureCB(moving_object_detector::MovingObjectDetectorConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: downsample_scale = %d, confidence_limit = %d, moving_flow_length = %f, flow_length_diff = %f, flow_start_diff = %f, flow_radian_diff = %f, flow_axis_max = %f, matching_tolerance = %f, cluster_element_num = %d", config.downsample_scale, config.confidence_limit, config.moving_flow_length, config.flow_length_diff, config.flow_start_diff, config.flow_radian_diff, config.flow_axis_max, config.matching_tolerance, config.cluster_element_num);
  
  downsample_scale_ = config.downsample_scale;
  confidence_limit_ = config.confidence_limit;
  moving_flow_length_ = config.moving_flow_length;
  flow_length_diff_ = config.flow_length_diff;
  flow_start_diff_ = config.flow_start_diff;
  flow_radian_diff_ = config.flow_radian_diff;
  flow_axis_max_ = config.flow_axis_max;
  matching_tolerance_ = config.matching_tolerance;
  cluster_element_num_ = config.cluster_element_num;
}

void MovingObjectDetector::dataCB(const geometry_msgs::TransformStampedConstPtr& camera_transform, const sensor_msgs::ImageConstPtr& optical_flow_left, const sensor_msgs::ImageConstPtr& optical_flow_right, const sensor_msgs::ImageConstPtr& depth_image_now, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& confidence_map, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  // 次の入力データをVISO2とflowノードに送信し，移動物体検出を行っている間に処理させる
  input_synchronizer_->publish();
  
  ros::Time start_process = ros::Time::now();
  camera_model_.fromCameraInfo(depth_image_info);
  pcl::PointCloud<pcl::PointXYZRGB> flow3d_pcl;
  pcl::PointCloud<pcl::PointXYZ> removed_by_matching;
  pcl::PointCloud<pcl::PointXYZ> removed_by_confidence;
  
  cv::Mat disparity_map_now = cv::Mat_<float>(disparity_image->image.height, disparity_image->image.width, (float*)&disparity_image->image.data[0], disparity_image->image.step);
  
  cv::Mat confidence_now;
  try
  {
    confidence_now = cv_bridge::toCvShare(confidence_map, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
    
  if (first_run_) {
    first_run_ = false;
  } else {
    std::list<std::list<Flow3D>> clusters; // clusters[cluster_index][cluster_element_index]
    
    // flow_mapの(x, y)には(dx,dy)が格納されている
    // つまり，フレームtの注目点の座標を(x,y)とすると，それに対応するフレームt-1の座標は(x-dx,y-dy)となる
    cv::Mat flow_map_left = cv_bridge::toCvShare(optical_flow_left)->image;
    cv::Mat flow_map_right = cv_bridge::toCvShare(optical_flow_right)->image;
    
    ros::Duration time_between_frames = camera_transform->header.stamp - time_stamp_previous_;
    
    int removed_by_confidence_error_num = 0; // 静止しているにも関わらず confidence によって除去されてしまった点の数
    
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
        if(!getPoint3D(left_now.x, left_now.y, camera_model_, *depth_image_now, point3d_now))
          continue;
        if(!getPoint3D(left_previous.x, left_previous.y, camera_model_, depth_image_previous_, point3d_previous))
          continue;
        
        if (left_now.x < 0 || left_now.x >= disparity_map_now.cols)
          continue;
        if (left_now.y < 0 || left_now.y >= disparity_map_now.rows)
          continue;
        
        float disparity_now = disparity_map_now.at<float>(left_now.y, left_now.x);
        if (std::isnan(disparity_now) || std::isinf(disparity_now) || disparity_now < 0)
          continue;
        
        if (left_previous.x < 0 || left_previous.x >= disparity_map_previous_.cols)
          continue;
        if (left_previous.y < 0 || left_previous.y >= disparity_map_previous_.rows)
          continue;
        
        float disparity_previous = disparity_map_previous_.at<float>(left_previous.y, left_previous.x);
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
        
        // 以前のフレームを現在のフレームに座標変換
        tf2::Stamped<tf2::Transform> tf_previous2now;
        tf2::fromMsg(*camera_transform, tf_previous2now);
        tf2::Vector3 point3d_previous_transformed = tf_previous2now * point3d_previous;
        
        Flow3D flow3d = Flow3D(point3d_previous_transformed, point3d_now, left_previous, left_now);
        
        if (left_previous.y < 0 || left_previous.y >= confidence_previous_.rows || left_previous.x < 0 || left_previous.x >= confidence_previous_.cols)
          continue;
        
        if (confidence_now.at<float>(left_now.y, left_now.x) > confidence_limit_ || confidence_previous_.at<float>(left_previous.y, left_previous.x) > confidence_limit_)
        {
          if (removed_by_confidence_pub_.getNumSubscribers() > 0)
          {
            pcl::PointXYZ pcl_point;
            pcl_point.x = flow3d.end.getX();
            pcl_point.y = flow3d.end.getY();
            pcl_point.z = flow3d.end.getZ();
            removed_by_confidence.push_back(pcl_point);
            
            if (flow3d.length() / time_between_frames.toSec() < moving_flow_length_)
              removed_by_confidence_error_num++;
          }
          continue;
        }
        
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = flow3d.end.getX();
        pcl_point.y = flow3d.end.getY();
        pcl_point.z = flow3d.end.getZ();
        
        tf2::Vector3 flow3d_vector = flow3d.distanceVector();
        uint32_t red, blue, green; 
        
        double flow_x_per_second = flow3d_vector.getX() / time_between_frames.toSec();
        red = std::abs(flow_x_per_second) / flow_axis_max_ * 255;
        if (red > 255)
          red = 255;
        
        double flow_y_per_second = flow3d_vector.getY() / time_between_frames.toSec();
        green = std::abs(flow_y_per_second) / flow_axis_max_ * 255;
        if (green > 255)
          green = 255;
        
        double flow_z_per_second = flow3d_vector.getZ() / time_between_frames.toSec();
        blue = std::abs(flow_z_per_second) / flow_axis_max_ * 255;
        if (blue > 255)
          blue = 255;
        
        uint32_t rgb = red << 16 | green << 8 | blue;
        pcl_point.rgb = *reinterpret_cast<float*>(&rgb);
        flow3d_pcl.push_back(pcl_point);
        
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
    
    sensor_msgs::PointCloud2 flow3d_msg;
    pcl::toROSMsg(flow3d_pcl, flow3d_msg);
    flow3d_msg.header.frame_id = depth_image_info->header.frame_id;
    flow3d_msg.header.stamp = depth_image_info->header.stamp;
    flow3d_pub_.publish(flow3d_msg);
    
    if (removed_by_confidence_pub_.getNumSubscribers() > 0)
    {
      double error_rate = 100.0 * removed_by_confidence_error_num / removed_by_confidence.size();
      confidence_error_rate_.push_back(error_rate);
      if (confidence_error_rate_.size() > 10)
        confidence_error_rate_.pop_front();
      double sum = 0;
      for (double rate : confidence_error_rate_)
        sum += rate;
      
      double average = sum / confidence_error_rate_.size();
      ROS_INFO("confidence average error rate in 10 frames[percent]: %f", average);
      
      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(removed_by_confidence, pointcloud_msg);
      pointcloud_msg.header.frame_id = depth_image_info->header.frame_id;
      pointcloud_msg.header.stamp = depth_image_info->header.stamp;
      removed_by_confidence_pub_.publish(pointcloud_msg);
    }
    
    if (removed_by_matching_pub_.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(removed_by_matching, pointcloud_msg);
      pointcloud_msg.header.frame_id = depth_image_info->header.frame_id;
      pointcloud_msg.header.stamp = depth_image_info->header.stamp;
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
      cluster_msg.header.frame_id = depth_image_info->header.frame_id;
      cluster_msg.header.stamp = depth_image_info->header.stamp;
      cluster_pub_.publish(cluster_msg);
    }
  }
  depth_image_previous_ = *depth_image_now;
  disparity_map_now.copyTo(disparity_map_previous_);
  confidence_now.copyTo(confidence_previous_);
  time_stamp_previous_ = camera_transform->header.stamp;
  
  ros::Duration process_time = ros::Time::now() - start_process;
  ROS_INFO("process time: %f", process_time.toSec());
}

MovingObjectDetector::InputSynchronizer::InputSynchronizer(MovingObjectDetector& outer_instance)
{
  image_transport_ = std::make_shared<image_transport::ImageTransport>(outer_instance.node_handle_);
  
  std::string publish_depth_image_topic = outer_instance.depth_image_sub_.getTopic();
  std::string publish_confidence_map_topic = outer_instance.confidence_map_sub_.getTopic();
  std::string publish_left_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_output_left_rect_image");
  std::string publish_right_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_output_right_rect_image");
  std::string publish_disparity_image_topic = outer_instance.disparity_image_sub_.getTopic();
  
  depth_image_pub_ = image_transport_->advertiseCamera(publish_depth_image_topic, 1);
  confidence_map_pub_ = outer_instance.node_handle_.advertise<sensor_msgs::Image>(publish_confidence_map_topic, 1);
  left_rect_image_pub_ = image_transport_->advertiseCamera(publish_left_rect_image_topic, 1);
  right_rect_image_pub_ = image_transport_->advertiseCamera(publish_right_rect_image_topic, 1);
  disparity_image_pub_ = outer_instance.node_handle_.advertise<stereo_msgs::DisparityImage>(publish_disparity_image_topic, 1);
  
  std::string subscribe_depth_image_topic = outer_instance.node_handle_.resolveName("synchronizer_input_depth_image");
  std::string subscribe_left_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_input_left_rect_image");
  std::string subscribe_right_rect_image_topic = outer_instance.node_handle_.resolveName("synchronizer_input_right_rect_image");
  
  depth_image_sub_.subscribe(outer_instance.node_handle_, subscribe_depth_image_topic, 10);
  depth_info_sub_.subscribe(outer_instance.node_handle_, image_transport::getCameraInfoTopic(subscribe_depth_image_topic), 10);
  confidence_map_sub_.subscribe(outer_instance.node_handle_, "synchronizer_input_confidence_map", 10);
  left_rect_image_sub_.subscribe(*image_transport_, subscribe_left_rect_image_topic, 10);
  left_rect_info_sub_.subscribe(outer_instance.node_handle_, image_transport::getCameraInfoTopic(subscribe_left_rect_image_topic), 10);
  right_rect_image_sub_.subscribe(*image_transport_, subscribe_right_rect_image_topic, 10);
  right_rect_info_sub_.subscribe(outer_instance.node_handle_, image_transport::getCameraInfoTopic(subscribe_right_rect_image_topic), 10);
  disparity_image_sub_.subscribe(outer_instance.node_handle_, "synchronizer_input_disparity_image", 10);
  time_sync_ = std::make_shared<DataTimeSynchronizer>(depth_image_sub_, depth_info_sub_, confidence_map_sub_, left_rect_image_sub_, left_rect_info_sub_, right_rect_image_sub_, right_rect_info_sub_, disparity_image_sub_, 10);
  time_sync_->registerCallback(boost::bind(&MovingObjectDetector::InputSynchronizer::dataCallBack, this, _1, _2, _3, _4, _5, _6, _7, _8));
}

void MovingObjectDetector::InputSynchronizer::dataCallBack(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_image_info, const sensor_msgs::ImageConstPtr& confidence_map, const sensor_msgs::ImageConstPtr& left_rect_image, const sensor_msgs::CameraInfoConstPtr& left_rect_info, const sensor_msgs::ImageConstPtr& right_rect_image, const sensor_msgs::CameraInfoConstPtr& right_rect_info, const stereo_msgs::DisparityImageConstPtr& disparity_image)
{
  static int count = 0;
  
  depth_image_ = *depth_image;
  depth_image_info_ = *depth_image_info;
  confidence_map_ = *confidence_map;
  left_rect_image_ = *left_rect_image;
  left_rect_info_ = *left_rect_info;
  right_rect_image_ = *right_rect_image;
  right_rect_info_ = *right_rect_info;
  disparity_image_ = *disparity_image;
  
  // MovingObjectDetector内でinput_synchronizer->publish()を行わない限り処理は開始しないので，初期2フレーム分のデータを送信する
  if (count < 2) {
    publish();
    count++;
  } else if ((ros::Time::now() - last_publish_time_).toSec() > 0.5) { // publishが途中で停止した場合には復帰
    publish();
  }
}

void MovingObjectDetector::InputSynchronizer::publish()
{
  depth_image_pub_.publish(depth_image_, depth_image_info_);
  confidence_map_pub_.publish(confidence_map_);
  left_rect_image_pub_.publish(left_rect_image_, left_rect_info_);
  right_rect_image_pub_.publish(right_rect_image_, right_rect_info_);
  disparity_image_pub_.publish(disparity_image_);
  
  last_publish_time_ = ros::Time::now();
}
