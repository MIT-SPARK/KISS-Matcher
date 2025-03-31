#pragma once

#ifndef KISS_MATCHER_LOOP_CLOSURE_H
#define KISS_MATCHER_LOOP_CLOSURE_H

///// C++ common headers
#include <iostream>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

#include "rclcpp/rclcpp.hpp"
#include "slam/loop_types.hpp"
#include "slam/pose_graph_node.hpp"
#include "slam/utils.hpp"

using NodePair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

namespace kiss_matcher {
struct GICPConfig {
  int num_threads_               = 4;
  int correspondence_randomness_ = 20;
  int max_num_iter_              = 20;

  double max_corr_dist_              = 1.0;
  double scale_factor_for_corr_dist_ = 5.0;
  double overlap_threshold_          = 90.0;
};

struct LoopClosureConfig {
  bool verbose_                    = false;
  bool enable_global_registration_ = true;
  bool is_multilayer_env_          = false;
  size_t num_submap_keyframes_     = 11;
  size_t num_inliers_threshold_    = 100;
  double voxel_res_                = 0.1;
  double loop_detection_radius_;
  double loop_detection_timediff_threshold_;
  GICPConfig gicp_config_;
  KISSMatcherConfig matcher_config_;
};

// Registration Output
struct RegOutput {
  bool is_valid_        = false;
  bool is_converged_    = false;
  double score_         = std::numeric_limits<double>::max();
  double overlapness_   = 0.0;
  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
};

class LoopClosure {
 private:
  // For coarse-to-fine alignment
  std::shared_ptr<kiss_matcher::KISSMatcher> global_reg_handler_                        = nullptr;
  std::shared_ptr<small_gicp::RegistrationPCL<PointType, PointType>> local_reg_handler_ = nullptr;

  pcl::PointCloud<PointType>::Ptr src_cloud_;
  pcl::PointCloud<PointType>::Ptr tgt_cloud_;
  pcl::PointCloud<PointType>::Ptr coarse_aligned_;
  pcl::PointCloud<PointType>::Ptr aligned_;
  pcl::PointCloud<PointType>::Ptr debug_cloud_;
  LoopClosureConfig config_;

  rclcpp::Logger logger_;

  std::chrono::steady_clock::time_point last_success_icp_time_;
  bool has_success_icp_time_ = false;

 public:
  explicit LoopClosure(const LoopClosureConfig &config, const rclcpp::Logger &logger);
  ~LoopClosure();
  double calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
  LoopCandidates getLoopCandidatesFromQuery(const PoseGraphNode &query_frame,
                                            const std::vector<PoseGraphNode> &keyframes);

  LoopCandidate getClosestCandidate(const LoopCandidates &candidates);

  LoopIdxPairs fetchClosestLoopCandidate(const PoseGraphNode &query_frame,
                                         const std::vector<PoseGraphNode> &keyframes);
  LoopIdxPairs fetchLoopCandidates(const PoseGraphNode &query_frame,
                                   const std::vector<PoseGraphNode> &keyframes,
                                   const size_t num_max_candidates  = 3,
                                   const double reliable_window_sec = 30);

  NodePair setSrcAndTgtCloud(const std::vector<PoseGraphNode> &keyframes,
                             const size_t src_idx,
                             const size_t tgt_idx,
                             const size_t num_submap_keyframes,
                             const double voxel_res,
                             const bool enable_global_registration);
  RegOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                         const pcl::PointCloud<PointType> &tgt);
  RegOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                  const pcl::PointCloud<PointType> &tgt);
  RegOutput performLoopClosure(const PoseGraphNode &query_keyframe,
                               const std::vector<PoseGraphNode> &keyframes);
  RegOutput performLoopClosure(const std::vector<PoseGraphNode> &keyframes,
                               const size_t query_idx,
                               const size_t match_idx);

  pcl::PointCloud<PointType> getSourceCloud();
  pcl::PointCloud<PointType> getTargetCloud();
  pcl::PointCloud<PointType> getCoarseAlignedCloud();
  pcl::PointCloud<PointType> getFinalAlignedCloud();
  pcl::PointCloud<PointType> getDebugCloud();
};
}  // namespace kiss_matcher
#endif  // KISS_MATCHER_LOOP_CLOSURE_H
