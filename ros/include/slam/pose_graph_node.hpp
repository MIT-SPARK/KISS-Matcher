#pragma once

#ifndef KISS_MATCHER_POSE_GRAPH_NODE_HPP
#define KISS_MATCHER_POSE_GRAPH_NODE_HPP

#include <string>

#include "slam/utils.hpp"

namespace kiss_matcher {

struct PoseGraphNode {
  pcl::PointCloud<PointType> scan_;
  pcl::PointCloud<PointType> voxelized_scan_;  // Used for map visualization
  Eigen::Matrix4d pose_           = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_ = Eigen::Matrix4d::Identity();
  double timestamp_;
  size_t idx_;
  bool nnsearch_processed_      = false;
  bool loop_detector_processed_ = false;

  PoseGraphNode() {}
  inline PoseGraphNode(const nav_msgs::msg::Odometry &odom,
                       const sensor_msgs::msg::PointCloud2 &scan,
                       const size_t idx,
                       const bool is_wrt_lidar_frame = false) {
    tf2::Quaternion q;
    q.setX(odom.pose.pose.orientation.x);
    q.setY(odom.pose.pose.orientation.y);
    q.setZ(odom.pose.pose.orientation.z);
    q.setW(odom.pose.pose.orientation.w);

    tf2::Matrix3x3 rot_tf(q);
    Eigen::Matrix3d rot;
    matrixTF2ToEigen(rot_tf, rot);

    pose_.block<3, 3>(0, 0) = rot;
    pose_(0, 3)             = odom.pose.pose.position.x;
    pose_(1, 3)             = odom.pose.pose.position.y;
    pose_(2, 3)             = odom.pose.pose.position.z;

    // This will be updated after pose graph optimization
    pose_corrected_ = pose_;

    pcl::PointCloud<PointType> scan_tmp;
    pcl::fromROSMsg(scan, scan_tmp);

    if (is_wrt_lidar_frame) {
      scan_ = scan_tmp;
    } else {
      pcl::PointCloud<PointType> pcd_wrt_lidar_frame = transformPcd(scan_tmp, pose_.inverse());
      scan_                                          = pcd_wrt_lidar_frame;
    }

    // ROS2 stamp.sec / stamp.nanosec
    timestamp_ = odom.header.stamp.sec + 1e-9 * odom.header.stamp.nanosec;
    idx_       = idx;
  }
};
}  // namespace kiss_matcher

#endif  // KISS_MATCHER_POSE_GRAPH_NODE_HPP
