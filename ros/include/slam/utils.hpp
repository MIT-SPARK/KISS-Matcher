#pragma once

#ifndef KISS_MATCHER_UTILS_HPP
#define KISS_MATCHER_UTILS_HPP

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using PointType = pcl::PointXYZI;

namespace kiss_matcher {
inline void matrixEigenToTF2(const Eigen::Matrix3d &in, tf2::Matrix3x3 &out) {
  out.setValue(
      in(0, 0), in(0, 1), in(0, 2), in(1, 0), in(1, 1), in(1, 2), in(2, 0), in(2, 1), in(2, 2));
}

inline void matrixTF2ToEigen(const tf2::Matrix3x3 &in, Eigen::Matrix3d &out) {
  out << in[0][0], in[0][1], in[0][2], in[1][0], in[1][1], in[1][2], in[2][0], in[2][1], in[2][2];
}

inline pcl::PointCloud<PointType>::Ptr voxelize(const pcl::PointCloud<PointType> &cloud,
                                                float voxel_res) {
  pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);

  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);

  cloud_ptr->reserve(cloud.size());
  cloud_out->reserve(cloud.size());
  *cloud_ptr = cloud;

  voxelgrid.setInputCloud(cloud_ptr);
  voxelgrid.filter(*cloud_out);
  return cloud_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelize(const pcl::PointCloud<PointType>::Ptr &cloud,
                                                float voxel_res) {
  pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);

  pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
  cloud_out->reserve(cloud->size());

  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*cloud_out);
  return cloud_out;
}

inline gtsam::Pose3 eigenToGtsam(const Eigen::Matrix4d &pose) {
  tf2::Matrix3x3 mat_tf;
  matrixEigenToTF2(pose.block<3, 3>(0, 0), mat_tf);

  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);

  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                      gtsam::Point3(pose(0, 3), pose(1, 3), pose(2, 3)));
}

inline Eigen::Matrix4d gtsamToEigen(const gtsam::Pose3 &pose) {
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();

  tf2::Quaternion quat;
  quat.setRPY(pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());

  tf2::Matrix3x3 tf_rot(quat);
  Eigen::Matrix3d rot;
  matrixTF2ToEigen(tf_rot, rot);

  output.block<3, 3>(0, 0) = rot;
  output(0, 3)             = pose.translation().x();
  output(1, 3)             = pose.translation().y();
  output(2, 3)             = pose.translation().z();

  return output;
}

inline geometry_msgs::msg::PoseStamped eigenToPoseStamped(const Eigen::Matrix4d &pose,
                                                          const std::string &frame_id = "map") {
  tf2::Matrix3x3 mat_tf;
  matrixEigenToTF2(pose.block<3, 3>(0, 0), mat_tf);

  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id    = frame_id;
  msg.pose.position.x    = pose(0, 3);
  msg.pose.position.y    = pose(1, 3);
  msg.pose.position.z    = pose(2, 3);
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();

  return msg;
}

inline geometry_msgs::msg::Pose egienToGeoPose(const Eigen::Matrix4d &pose) {
  tf2::Matrix3x3 mat_tf;
  matrixEigenToTF2(pose.block<3, 3>(0, 0), mat_tf);

  double roll, pitch, yaw;
  mat_tf.getRPY(roll, pitch, yaw);

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Pose msg;
  msg.position.x    = pose(0, 3);
  msg.position.y    = pose(1, 3);
  msg.position.z    = pose(2, 3);
  msg.orientation.w = quat.w();
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();

  return msg;
}

inline geometry_msgs::msg::PoseStamped gtsamToPoseStamped(const gtsam::Pose3 &pose,
                                                          const std::string &frame_id = "map") {
  double roll  = pose.rotation().roll();
  double pitch = pose.rotation().pitch();
  double yaw   = pose.rotation().yaw();

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id    = frame_id;
  msg.pose.position.x    = pose.translation().x();
  msg.pose.position.y    = pose.translation().y();
  msg.pose.position.z    = pose.translation().z();
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();

  return msg;
}

template <typename T>
inline sensor_msgs::msg::PointCloud2 toROSMsg(const pcl::PointCloud<T> &cloud,
                                              const std::string &frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ros;
  pcl::toROSMsg(cloud, cloud_ros);
  cloud_ros.header.frame_id = frame_id;
  return cloud_ros;
}

template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose) {
  if (cloud_in.empty()) {
    return cloud_in;
  }
  pcl::PointCloud<T> cloud_out;
  pcl::transformPointCloud(cloud_in, cloud_out, pose);
  return cloud_out;
}

template <typename T>
inline std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<T> &cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto &pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}
}  // namespace kiss_matcher
#endif  // KISS_MATCHER_UTILS_HPP
