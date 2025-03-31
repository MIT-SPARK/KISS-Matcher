#include "slam/loop_closure.h"

using namespace kiss_matcher;

LoopClosure::LoopClosure(const LoopClosureConfig &config, const rclcpp::Logger &logger)
    : config_(config), logger_(logger) {
  config_.matcher_config_ = kiss_matcher::KISSMatcherConfig(config_.voxel_res_, false);

  auto &gc          = config_.gicp_config_;
  gc.max_corr_dist_ = config_.voxel_res_ * gc.scale_factor_for_corr_dist_;

  src_cloud_.reset(new pcl::PointCloud<PointType>());
  tgt_cloud_.reset(new pcl::PointCloud<PointType>());
  coarse_aligned_.reset(new pcl::PointCloud<PointType>());
  aligned_.reset(new pcl::PointCloud<PointType>());
  debug_cloud_.reset(new pcl::PointCloud<PointType>());

  global_reg_handler_ = std::make_shared<kiss_matcher::KISSMatcher>(config_.matcher_config_);
  local_reg_handler_  = std::make_shared<small_gicp::RegistrationPCL<PointType, PointType>>();

  local_reg_handler_->setNumThreads(gc.num_threads_);
  local_reg_handler_->setCorrespondenceRandomness(gc.correspondence_randomness_);
  local_reg_handler_->setMaxCorrespondenceDistance(gc.max_corr_dist_);
  local_reg_handler_->setVoxelResolution(config.voxel_res_);
  local_reg_handler_->setRegistrationType("VGICP");  // "VGICP" or "GICP"
}

LoopClosure::~LoopClosure() {}

// NOTE(hlim): In outdoor scenes, loop closure sometimes fails due to Z-axis drift.
// To address this, we use the is_multilayer_env_ parameter.
// If true, full 3D distance (including Z) is considered for loop detection.
// If false, we ignore Z and compute distance on the XY plane only.
double LoopClosure::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2) {
  if (config_.is_multilayer_env_) {
    return (pose1.block<3, 1>(0, 3) - pose2.block<3, 1>(0, 3)).norm();
  } else {
    return (pose1.block<2, 1>(0, 3) - pose2.block<2, 1>(0, 3)).norm();
  }
}

LoopCandidates LoopClosure::getLoopCandidatesFromQuery(
    const PoseGraphNode &query_frame,
    const std::vector<PoseGraphNode> &keyframes) {
  LoopCandidates candidates;
  candidates.reserve(keyframes.size() / 100);  // heuristic: expect ~1% to be valid

  const auto &loop_det_radi      = config_.loop_detection_radius_;
  const auto &loop_det_tdiff_thr = config_.loop_detection_timediff_threshold_;

  for (size_t idx = 0; idx < keyframes.size() - 1; ++idx) {
    double dist = calculateDistance(keyframes[idx].pose_corrected_, query_frame.pose_corrected_);
    double time_diff = query_frame.timestamp_ - keyframes[idx].timestamp_;

    if (dist < loop_det_radi && time_diff > loop_det_tdiff_thr) {
      LoopCandidate c;
      c.idx_      = keyframes[idx].idx_;
      c.distance_ = dist;
      c.found_    = true;
      candidates.emplace_back(c);
    }
  }

  return candidates;
}

LoopCandidate LoopClosure::getClosestCandidate(const LoopCandidates &candidates) {
  if (candidates.empty()) return LoopCandidate();

  return *std::min_element(
      candidates.begin(), candidates.end(), [](const LoopCandidate &a, const LoopCandidate &b) {
        return a.distance_ < b.distance_;
      });
}

LoopIdxPairs LoopClosure::fetchClosestLoopCandidate(const PoseGraphNode &query_frame,
                                                    const std::vector<PoseGraphNode> &keyframes) {
  const auto &candidates = getLoopCandidatesFromQuery(query_frame, keyframes);
  if (candidates.empty()) {
    return {};
  }

  const auto &candidate = getClosestCandidate(candidates);
  // NOTE(hlim): While it outputs a single index pair,
  // for compatabiliy, I decided to use `LoopIdxPairs` instead of `LoopIdxPair`.
  LoopIdxPairs idx_pairs;
  idx_pairs.emplace_back(query_frame.idx_, candidate.idx_);
  return idx_pairs;
}

LoopIdxPairs LoopClosure::fetchLoopCandidates(const PoseGraphNode &query_frame,
                                              const std::vector<PoseGraphNode> &keyframes,
                                              const size_t num_max_candidates,
                                              const double reliable_window_sec) {
  // Case A.
  // If ICP was recently successful, it is highly likely that pose graph optimization (PGO)
  // has corrected the poses. As a result, the nearest neighbor becomes a much more reliable
  // loop candidate, and a single closest candidate is often sufficient.
  if (has_success_icp_time_) {
    auto now           = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(now - last_success_icp_time_).count();

    if (elapsed_sec < reliable_window_sec) {
      if (config_.verbose_) {
        RCLCPP_INFO(logger_, "The nearest loop candidate is returned.");
      }
      return fetchClosestLoopCandidate(query_frame, keyframes);
    }
  }

  // Case B.
  // If no loop closure has occurred for a long time, the nearest neighbor is not always
  // a valid loop candidate due to possible drift or noise.
  // To mitigate this, we use random sampling to diversify the candidates.
  LoopCandidates candidates = getLoopCandidatesFromQuery(query_frame, keyframes);
  if (candidates.empty()) {
    return {};
  }

  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(candidates.begin(), candidates.end(), g);

  LoopIdxPairs idx_pairs;
  size_t num_selected = std::min(num_max_candidates, candidates.size());
  for (size_t i = 0; i < num_selected; ++i) {
    idx_pairs.emplace_back(query_frame.idx_, candidates[i].idx_);
  }

  return idx_pairs;
}

NodePair LoopClosure::setSrcAndTgtCloud(const std::vector<PoseGraphNode> &keyframes,
                                        const size_t src_idx,
                                        const size_t tgt_idx,
                                        const size_t num_submap_keyframes,
                                        const double voxel_res,
                                        const bool enable_global_registration) {
  const size_t submap_range = num_submap_keyframes / 2;
  const size_t num_approx   = keyframes[src_idx].scan_.size() * num_submap_keyframes;

  pcl::PointCloud<PointType> tgt_accum, src_accum;
  src_accum.reserve(num_approx);
  tgt_accum.reserve(num_approx);

  const bool build_submap = (num_submap_keyframes > 1);

  auto accumulateSubmap = [&](size_t center_idx, pcl::PointCloud<PointType> &accum) {
    const size_t start = (center_idx < submap_range) ? 0 : center_idx - submap_range;
    const size_t end   = std::min(center_idx + submap_range + 1, keyframes.size());
    for (size_t i = start; i < end; ++i) {
      accum += transformPcd(keyframes[i].scan_, keyframes[i].pose_corrected_);
    }
  };

  if (build_submap) {
    accumulateSubmap(src_idx, src_accum);
    accumulateSubmap(tgt_idx, tgt_accum);
  } else {
    src_accum = transformPcd(keyframes[src_idx].scan_, keyframes[src_idx].pose_corrected_);
    if (enable_global_registration) {
      tgt_accum = transformPcd(keyframes[tgt_idx].scan_, keyframes[tgt_idx].pose_corrected_);
    } else {
      // For ICP matching,
      // empirically scan-to-submap matching works better than scan-to-scan matching
      accumulateSubmap(tgt_idx, tgt_accum);
    }
  }
  return {*voxelize(src_accum, voxel_res), *voxelize(tgt_accum, voxel_res)};
}

RegOutput LoopClosure::icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &tgt) {
  RegOutput reg_output;
  aligned_->clear();
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>());
  *src_cloud = src;
  *tgt_cloud = tgt;
  local_reg_handler_->setInputTarget(tgt_cloud);
  local_reg_handler_->setInputSource(src_cloud);

  local_reg_handler_->align(*aligned_);

  const auto &local_reg_result = local_reg_handler_->getRegistrationResult();

  double overlapness =
      static_cast<double>(local_reg_result.num_inliers) / src_cloud->size() * 100.0;
  reg_output.overlapness_ = overlapness;

  // NOTE(hlim): fine_T_coarse
  reg_output.pose_ = local_reg_handler_->getFinalTransformation().cast<double>();
  // if matchness overlapness is over than threshold,
  // that means the registration result is likely to be sufficiently overlapped
  if (overlapness > config_.gicp_config_.overlap_threshold_) {
    reg_output.is_valid_     = true;
    reg_output.is_converged_ = true;

    last_success_icp_time_ = std::chrono::steady_clock::now();
    has_success_icp_time_  = true;
  }
  if (config_.verbose_) {
    if (overlapness > config_.gicp_config_.overlap_threshold_) {
      RCLCPP_INFO(logger_,
                  "Overlapness: \033[1;32m%.2f%% > %.2f%%\033[0m",
                  overlapness,
                  config_.gicp_config_.overlap_threshold_);
    } else {
      RCLCPP_WARN(logger_,
                  "Overlapness: %.2f%% < %.2f%%\033[0m",
                  overlapness,
                  config_.gicp_config_.overlap_threshold_);
    }
  }
  return reg_output;
}

RegOutput LoopClosure::coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &tgt) {
  RegOutput reg_output;
  coarse_aligned_->clear();

  const auto &src_vec = convertCloudToVec(src);
  const auto &tgt_vec = convertCloudToVec(tgt);

  const auto &solution = global_reg_handler_->estimate(src_vec, tgt_vec);

  Eigen::Matrix4d coarse_alignment      = Eigen::Matrix4d::Identity();
  coarse_alignment.block<3, 3>(0, 0)    = solution.rotation.cast<double>();
  coarse_alignment.topRightCorner(3, 1) = solution.translation.cast<double>();

  *coarse_aligned_ = transformPcd(src, coarse_alignment);

  const size_t num_inliers = global_reg_handler_->getNumFinalInliers();
  if (config_.verbose_) {
    if (num_inliers > config_.num_inliers_threshold_) {
      RCLCPP_INFO(logger_,
                  "\033[1;32m# final inliers: %lu > %lu\033[0m",
                  num_inliers,
                  config_.num_inliers_threshold_);
    } else {
      RCLCPP_WARN(
          logger_, "# final inliers: %lu < %lu", num_inliers, config_.num_inliers_threshold_);
    }
  }

  // NOTE(hlim): A small number of inliers suggests that the initial alignment may have failed,
  // so fine alignment is meaningless.
  if (!solution.valid || num_inliers < config_.num_inliers_threshold_) {
    return reg_output;
  } else {
    const auto &fine_output = icpAlignment(*coarse_aligned_, tgt);
    reg_output              = fine_output;
    reg_output.pose_        = fine_output.pose_ * coarse_alignment;

    // Use this cloud to debug whether the transformation is correct.
    // *debug_cloud_        = transformPcd(src, reg_output.pose_);
  }
  return reg_output;
}

RegOutput LoopClosure::performLoopClosure(const PoseGraphNode &query_keyframe,
                                          const std::vector<PoseGraphNode> &keyframes) {
  const auto &loop_candidate = fetchClosestLoopCandidate(query_keyframe, keyframes);
  if (loop_candidate.empty()) {
    return RegOutput();
  }
  const auto &[query_idx, match_idx] = loop_candidate[0];
  return performLoopClosure(keyframes, query_idx, match_idx);
}

RegOutput LoopClosure::performLoopClosure(const std::vector<PoseGraphNode> &keyframes,
                                          const size_t query_idx,
                                          const size_t match_idx) {
  RegOutput reg_output;
  if (match_idx >= 0) {
    const auto &[src_cloud, tgt_cloud] = setSrcAndTgtCloud(keyframes,
                                                           query_idx,
                                                           match_idx,
                                                           config_.num_submap_keyframes_,
                                                           config_.voxel_res_,
                                                           config_.enable_global_registration_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *tgt_cloud_ = tgt_cloud;

    if (config_.enable_global_registration_) {
      RCLCPP_INFO(logger_,
                  "\033[1;35mExecute coarse-to-fine alignment: # src = %lu, # tgt = %lu\033[0m",
                  src_cloud.size(),
                  tgt_cloud.size());
      return coarseToFineAlignment(src_cloud, tgt_cloud);
    } else {
      RCLCPP_INFO(logger_,
                  "\033[1;35mExecute GICP: # src = %lu, # tgt = %lu\033[0m",
                  src_cloud.size(),
                  tgt_cloud.size());
      return icpAlignment(src_cloud, tgt_cloud);
    }
  } else {
    return reg_output;
  }
}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud() { return *src_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getTargetCloud() { return *tgt_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud() { return *coarse_aligned_; }

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud() { return *aligned_; }

pcl::PointCloud<PointType> LoopClosure::getDebugCloud() { return *debug_cloud_; }
