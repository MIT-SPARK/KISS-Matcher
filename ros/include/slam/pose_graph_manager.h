#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_MANAGER_H
#define KISS_MATCHER_POSE_GRAPH_MANAGER_H

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

// message_filters in ROS2
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "slam/loop_closure.h"
#include "slam/loop_detector.h"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

// #include <pose_graph_tools_msgs/msg/pose_graph.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                        sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

class PoseGraphManager : public rclcpp::Node {
 public:
  PoseGraphManager() = delete;
  explicit PoseGraphManager(const rclcpp::NodeOptions &options);
  ~PoseGraphManager();

 private:
  void updateOdomsAndPaths(const kiss_matcher::PoseGraphNode &pose_pcd_in);
  bool checkIfKeyframe(const kiss_matcher::PoseGraphNode &pose_pcd_in,
                       const kiss_matcher::PoseGraphNode &latest_pose_pcd);
  visualization_msgs::msg::Marker visualizeLoopMarkers(const gtsam::Values &corrected_poses) const;
  visualization_msgs::msg::Marker visualizeLoopDetectionRadius(
      const geometry_msgs::msg::Point &latest_position) const;

  void callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
  void saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
  /**** Timer functions ****/
  // void loopPubTimerFunc();
  void buildMap();
  void detectLoopClosureByLoopDetector();
  void detectLoopClosureByNNSearch();
  void publishVisualization();

  std::string map_frame_;
  std::string base_frame_;
  std::string package_path_;
  std::string seq_name_;

  std::mutex realtime_pose_mutex_;
  std::mutex keyframes_mutex_;
  std::mutex graph_mutex_;
  std::mutex vis_mutex_;

  Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_delta_          = Eigen::Matrix4d::Identity();
  kiss_matcher::PoseGraphNode current_frame_;
  std::vector<kiss_matcher::PoseGraphNode> keyframes_;
  int current_keyframe_idx_ = 0;

  bool is_initialized_                        = false;
  bool loop_added_flag_                       = false;
  bool loop_added_flag_map_                   = false;
  bool loop_added_flag_vis_                   = false;
  std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
  gtsam::NonlinearFactorGraph gtsam_graph_;
  gtsam::Values init_esti_;
  gtsam::Values corrected_esti_;

  double keyframe_thr_;
  double scan_voxel_res_;
  double map_voxel_res_;
  double save_voxel_res_;
  double loop_pub_delayed_time_;
  double loop_detection_radius_;  // Only for visualization
  int sub_key_num_;
  std::vector<std::pair<size_t, size_t>> loop_idx_pairs_;
  // pose_graph_tools_msgs::msg::PoseGraph loop_msgs_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
  nav_msgs::msg::Path odom_path_, corrected_path_;

  bool save_map_bag_         = false;
  bool save_map_pcd_         = false;
  bool save_in_kitti_format_ = false;
  double last_lc_time_       = 0.0;

  std::shared_ptr<kiss_matcher::LoopClosure> loop_closure_;

  // NOTE(hlim): We do not provide a loop detector implementation directly,
  // but you can plug in your own detector via this interface.
  std::shared_ptr<kiss_matcher::LoopDetector> loop_detector_;

  pcl::PointCloud<PointType>::Ptr map_cloud_;

  // ROS2 interface
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_radius_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_tgt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;

  // rclcpp::Publisher<pose_graph_tools_msgs::msg::PoseGraph>::SharedPtr loop_closures_pub_;

  // message_filters
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_scan_;
  std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_node_;

  // Timers
  rclcpp::TimerBase::SharedPtr hydra_loop_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr loop_detector_timer_;
  rclcpp::TimerBase::SharedPtr loop_nnsearch_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
};

#endif  // KISS_MATCHER_POSE_GRAPH_MANAGER_H
