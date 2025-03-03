/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include <kiss_matcher/KISSMatcher.hpp>

namespace kiss_matcher {
KISSMatcher::KISSMatcher(const float &voxel_size) { config_ = KISSMatcherConfig(voxel_size); }

KISSMatcher::KISSMatcher(const KISSMatcherConfig &config) {
  config_ = config;
  reset();
}

void KISSMatcher::reset() {
  faster_pfh_ = std::make_unique<FasterPFH>(
      config_.normal_radius_, config_.fpfh_radius_, config_.thr_linearity_);
  robin_matching_ = std::make_unique<ROBINMatching>(
      config_.robin_noise_bound_, config_.num_max_corr_, config_.tuple_scale_);

  resetSolver();
}

void KISSMatcher::resetSolver() {
  // NOTE(hlim) Please turn on `use_quatro_`
  // when the pitch and roll angles are not dominant in the rotation
  kiss_matcher::RobustRegistrationSolver::Params params;
  params.noise_bound = config_.solver_noise_bound_;

  if (config_.use_quatro_) {
    params.rotation_estimation_algorithm =
        kiss_matcher::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  } else {
    params.rotation_estimation_algorithm =
        kiss_matcher::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  }

  solver_ = std::make_unique<RobustRegistrationSolver>(params);
}

kiss_matcher::KeypointPair KISSMatcher::match(const std::vector<Eigen::Vector3f> &src,
                                              const std::vector<Eigen::Vector3f> &tgt) {
  clear();
  auto processInput = [&](const std::vector<Eigen::Vector3f> &input_cloud) {
    if (config_.use_voxel_sampling_) {
      return VoxelgridSampling(input_cloud, config_.voxel_size_);
    }
    return input_cloud;
  };

  auto t_init = std::chrono::high_resolution_clock::now();

  src_processed_ = std::move(processInput(src));
  tgt_processed_ = std::move(processInput(tgt));

  auto t_process = std::chrono::high_resolution_clock::now();

  faster_pfh_->setInputCloud(src_processed_);
  // Note(hlim) Some erroneous points are filtered out
  // Thus, # of `src_keypoints_` <= `src_processed_`
  faster_pfh_->ComputeFeature(src_keypoints_, src_descriptors_);

  faster_pfh_->setInputCloud(tgt_processed_);
  // Note(hlim) Some erroneous points are filtered out
  // Thus, # of `tgt_keypoints_` <= `tgt_processed_`
  faster_pfh_->ComputeFeature(tgt_keypoints_, tgt_descriptors_);

  auto t_mid = std::chrono::high_resolution_clock::now();

  const auto &corr = robin_matching_->establishCorrespondences(src_keypoints_,
                                                               tgt_keypoints_,
                                                               src_descriptors_,
                                                               tgt_descriptors_,
                                                               config_.robin_mode_,
                                                               config_.tuple_scale_,
                                                               config_.use_ratio_test_);

  src_matched_.resize(corr.size());
  tgt_matched_.resize(corr.size());

  for (size_t i = 0; i < corr.size(); ++i) {
    auto src_idx    = std::get<0>(corr[i]);
    auto dst_idx    = std::get<1>(corr[i]);
    src_matched_[i] = src_keypoints_[src_idx];
    tgt_matched_[i] = tgt_keypoints_[dst_idx];
  }
  auto t_end = std::chrono::high_resolution_clock::now();

  processing_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_process - t_init).count();
  extraction_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_mid - t_process).count();
  matching_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_mid).count();

  return {src_matched_, tgt_matched_};
}

kiss_matcher::KeypointPair KISSMatcher::match(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src,
                                              const Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt) {
  std::vector<Eigen::Vector3f> src_vec(src.cols());
  std::vector<Eigen::Vector3f> tgt_vec(tgt.cols());

  for (ssize_t i = 0; i < src.cols(); ++i) {
    src_vec[i] = src.col(i).cast<float>();
  }
  for (ssize_t i = 0; i < tgt.cols(); ++i) {
    tgt_vec[i] = tgt.col(i).cast<float>();
  }

  return match(src_vec, tgt_vec);
}

kiss_matcher::RegistrationSolution KISSMatcher::estimate(const std::vector<Eigen::Vector3f> &src,
                                                         const std::vector<Eigen::Vector3f> &tgt) {
  const auto &[src_matched, tgt_matched] = match(src, tgt);
  size_t M                               = src_matched.size();

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched_eigen;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched_eigen;

  src_matched_eigen.resize(3, M);
  tgt_matched_eigen.resize(3, M);
  for (size_t m = 0; m < M; ++m) {
    src_matched_eigen.col(m) << src_matched[m].cast<double>();
    tgt_matched_eigen.col(m) << tgt_matched[m].cast<double>();
  }
  return solve(src_matched_eigen, tgt_matched_eigen);
}

RegistrationSolution KISSMatcher::solve(
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &src_matched,
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt_matched) {
  // In case of too-few matching pairs,
  // Just return invalid solution with the identity matrix
  if (src_matched.cols() < 2) {
    return solver_->getSolution();
  }

  resetSolver();
  std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
  solver_->solve(src_matched, tgt_matched);
  std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
  solver_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();

  return solver_->getSolution();
}

RegistrationSolution KISSMatcher::pruneAndSolve(const std::vector<Eigen::Vector3f> &src_matched,
                                                const std::vector<Eigen::Vector3f> &tgt_matched) {
  std::vector<std::pair<int, int>> corres, corres_out;
  for (size_t i = 0; i < src_matched.size(); ++i) {
    corres.emplace_back(i, i);
  }
  const auto &pruned_indices =
      robin_matching_->applyOutlierPruning(src_matched, tgt_matched, "max_core");
  size_t num_pruned_corr = pruned_indices.size();

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_eigen(3, num_pruned_corr);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_eigen(3, num_pruned_corr);

  for (size_t i = 0; i < num_pruned_corr; ++i) {
    src_eigen.col(i) = src_matched[num_pruned_corr].cast<double>();
    tgt_eigen.col(i) = tgt_matched[num_pruned_corr].cast<double>();
  }
  return solve(src_eigen, tgt_eigen);
}

double KISSMatcher::getProcessingTime() { return processing_time_; }

double KISSMatcher::getExtractionTime() { return extraction_time_; }

double KISSMatcher::getRejectionTime() { return robin_matching_->getRejectionTime(); }

double KISSMatcher::getMatchingTime() { return matching_time_; }

double KISSMatcher::getSolverTime() { return solver_time_; }

void KISSMatcher::print() {
  const double t_p = getProcessingTime();
  const double t_e = getExtractionTime();
  const double t_r = getRejectionTime();
  const double t_m = getMatchingTime();
  const double t_s = getSolverTime();

  std::cout << "============== Time =============="
            << "\n";
  std::cout << "Voxelization: " << t_p << " sec\n";
  std::cout << "Extraction  : " << t_e << " sec\n";
  std::cout << "Pruning     : " << t_r << " sec\n";
  std::cout << "Matching    : " << t_m << " sec\n";
  std::cout << "Solving     : " << t_s << " sec\n";
  std::cout << "----------------------------------"
            << "\n";
  std::cout << "\033[1;32mTotal     : " << t_p + t_e + t_r + t_m + t_s << " sec\033[0m\n";
  std::cout << "====== # of correspondences ======"
            << "\n";
  std::cout << "# initial pairs : " << robin_matching_->getNumInitialCorrespondences() << "\n";
  std::cout << "# pruned pairs  : " << robin_matching_->getNumPrunedCorrespondences() << "\n";
  std::cout << "----------------------------------"
            << "\n";
  std::cout << "\033[1;36m# rot inliers   : " << solver_->getRotationInliers().size() << "\n";
  std::cout << "# trans inliers : " << solver_->getTranslationInliers().size() << "\033[0m\n";
  std::cout << "=================================="
            << "\n";
}
}  // namespace kiss_matcher
