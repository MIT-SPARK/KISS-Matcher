#include "slam/loop_closure.h"

using namespace kiss_matcher;

LoopClosure::LoopClosure(const LoopClosureConfig &config) {
  config_                 = config;
  config_.matcher_config_ = kiss_matcher::KISSMatcherConfig(config_.voxel_res_, false);

  auto &gc          = config_.gicp_config_;
  gc.max_corr_dist_ = config_.voxel_res_ * gc.scale_factor_for_corr_dist_;

  src_cloud_.reset(new pcl::PointCloud<PointType>());
  tgt_cloud_.reset(new pcl::PointCloud<PointType>());
  coarse_aligned_.reset(new pcl::PointCloud<PointType>());
  aligned_.reset(new pcl::PointCloud<PointType>());

  global_reg_handler_ = std::make_shared<kiss_matcher::KISSMatcher>(config_.matcher_config_);
  local_reg_handler_  = std::make_shared<small_gicp::RegistrationPCL<PointType, PointType>>();

  local_reg_handler_->setNumThreads(gc.num_threads_);
  local_reg_handler_->setCorrespondenceRandomness(gc.correspondence_randomness_);
  local_reg_handler_->setMaxCorrespondenceDistance(gc.max_corr_dist_);
  local_reg_handler_->setVoxelResolution(config.voxel_res_);
  local_reg_handler_->setRegistrationType("VGICP");  // "VGICP" or "GICP"
}

LoopClosure::~LoopClosure() {}

int LoopClosure::fetchClosestKeyframeIdx(const PoseGraphNode &front_keyframe,
                                         const std::vector<PoseGraphNode> &keyframes) {
  const auto &loop_det_radi      = config_.loop_detection_radius_;
  const auto &loop_det_tdiff_thr = config_.loop_detection_timediff_threshold_;
  double shortest_distance_      = loop_det_radi * 3.0;
  int closest_idx                = -1;
  for (size_t idx = 0; idx < keyframes.size() - 1; ++idx) {
    // check if potential loop: close enough in distance, far enough in time
    double tmp_dist = (keyframes[idx].pose_corrected_.block<3, 1>(0, 3) -
                       front_keyframe.pose_corrected_.block<3, 1>(0, 3))
                          .norm();
    if (loop_det_radi > tmp_dist &&
        loop_det_tdiff_thr < (front_keyframe.timestamp_ - keyframes[idx].timestamp_)) {
      if (tmp_dist < shortest_distance_) {
        shortest_distance_ = tmp_dist;
        closest_idx        = keyframes[idx].idx_;
      }
    }
  }
  return closest_idx;
}

NodePair LoopClosure::setSrcAndTgtCloud(const std::vector<PoseGraphNode> &keyframes,
                                        const int src_idx,
                                        const int tgt_idx,
                                        const int submap_range,
                                        const double voxel_res,
                                        const bool enable_global_registration,
                                        const bool enable_submap_matching) {
  pcl::PointCloud<PointType> tgt_accum, src_accum;
  int num_approx = keyframes[src_idx].scan_.size() * 2 * submap_range;
  src_accum.reserve(num_approx);
  tgt_accum.reserve(num_approx);
  if (enable_submap_matching) {
    for (int i = src_idx - submap_range; i < src_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)) {
        src_accum += transformPcd(keyframes[i].scan_, keyframes[i].pose_corrected_);
      }
    }
    for (int i = tgt_idx - submap_range; i < tgt_idx + submap_range + 1; ++i) {
      if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)) {
        tgt_accum += transformPcd(keyframes[i].scan_, keyframes[i].pose_corrected_);
      }
    }
  } else {
    src_accum = transformPcd(keyframes[src_idx].scan_, keyframes[src_idx].pose_corrected_);
    if (enable_global_registration) {
      tgt_accum = transformPcd(keyframes[tgt_idx].scan_, keyframes[tgt_idx].pose_corrected_);
    } else {
      // For ICP matching,
      // empirically scan-to-submap matching works better
      for (int i = tgt_idx - submap_range; i < tgt_idx + submap_range + 1; ++i) {
        if (i >= 0 && i < static_cast<int>(keyframes.size() - 1)) {
          tgt_accum += transformPcd(keyframes[i].scan_, keyframes[i].pose_corrected_);
        }
      }
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
  // if matchness overlapness is over than threshold,
  // that means the registration result is likely to be sufficiently overlapped
  if (overlapness > config_.gicp_config_.overlap_threshold_) {
    reg_output.is_valid_     = true;
    reg_output.is_converged_ = true;
    reg_output.pose_ = local_reg_handler_->getFinalTransformation().inverse().cast<double>();
  }
  if (config_.verbose_) {
    if (overlapness > config_.gicp_config_.overlap_threshold_) {
      std::cout << "\033[1;32m" << overlapness << "% > ";
    } else {
      std::cout << "\033[1;33m" << overlapness << "% < ";
    }
    std::cout << config_.gicp_config_.overlap_threshold_ << "%\033[1;0m\n";
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

  if (config_.verbose_) {
    std::cout << "# of final inliers: " << global_reg_handler_->getNumFinalInliers() << "\n";
  }

  if (!solution.valid) {
    return reg_output;
  } else {
    // coarse align with the result of Quatro
    *coarse_aligned_        = transformPcd(src, coarse_alignment);
    const auto &fine_output = icpAlignment(*coarse_aligned_, tgt);
    const auto quatro_tf_   = reg_output.pose_;
    reg_output              = fine_output;
    reg_output.pose_        = fine_output.pose_ * quatro_tf_;
  }
  return reg_output;
}

RegOutput LoopClosure::performLoopClosure(const PoseGraphNode &query_keyframe,
                                          const std::vector<PoseGraphNode> &keyframes) {
  closest_keyframe_idx_ = fetchClosestKeyframeIdx(query_keyframe, keyframes);
  return performLoopClosure(query_keyframe, keyframes, closest_keyframe_idx_);
}

RegOutput LoopClosure::performLoopClosure(const PoseGraphNode &query_keyframe,
                                          const std::vector<PoseGraphNode> &keyframes,
                                          const int closest_keyframe_idx) {
  RegOutput reg_output;
  closest_keyframe_idx_ = closest_keyframe_idx;
  if (closest_keyframe_idx_ >= 0) {
    const auto &[src_cloud, tgt_cloud] = setSrcAndTgtCloud(keyframes,
                                                           query_keyframe.idx_,
                                                           closest_keyframe_idx_,
                                                           config_.num_submap_keyframes_,
                                                           config_.voxel_res_,
                                                           config_.enable_global_registration_,
                                                           config_.enable_submap_matching_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *tgt_cloud_ = tgt_cloud;

    if (config_.enable_global_registration_) {
      std::cout << "\033[1;35mExecute coarse-to-fine alignment: " << src_cloud.size() << " vs "
                << tgt_cloud.size() << "\033[0m\n";
      return coarseToFineAlignment(src_cloud, tgt_cloud);
    } else {
      std::cout << "\033[1;35mExecute GICP: " << src_cloud.size() << " vs " << tgt_cloud.size()
                << "\033[0m\n";
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

int LoopClosure::getClosestKeyframeidx() { return closest_keyframe_idx_; }
