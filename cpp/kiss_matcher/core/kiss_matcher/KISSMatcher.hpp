/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */
#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "kiss_matcher/FasterPFH.hpp"
#include "kiss_matcher/GncSolver.hpp"
#include "kiss_matcher/ROBINMatching.hpp"
#include "kiss_matcher/points/downsampling.hpp"
#include "kiss_matcher/tsl/robin_map.h"

namespace kiss_matcher {
using KeypointPair = std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>;

struct KISSMatcherConfig {
  bool use_voxel_sampling_ = true;

  // FPFH descriptor params
  float voxel_size_    = 0.3;
  float normal_radius_ = 0.9;  // 2.5 * `voxel_size` - 3.0 * `voxel_size`
  float fpfh_radius_   = 1.5;  // 5.0 * `voxel_size`

  // Graph-theoretic outlier rejection parms
  float thr_linearity_ = 1.0;  // 1.0 means that we won't use linearity-based filtering
  // NOTE(hlim): The final `robin_noise_bound` becomes `voxel_size_` * `robin_noise_bound_gain_`
  float robin_noise_bound_gain_ = 1.0;
  float robin_noise_bound_      = voxel_size_ * robin_noise_bound_gain_;

  // matching params
  // NOTE(hlim): For better usability for map-level registration, I set `true` as a default
  // Enabling `use_ratio_test_` may cause a slight slowdown,
  // and its impact is insignificant at the scan level.
  bool use_ratio_test_    = true;
  std::string robin_mode_ = "max_core";
  float tuple_scale_      = 0.95;
  int num_max_corr_       = 5000;

  // Solver params
  // NOTE(hlim): The final `solver_noise_bound` becomes `voxel_size_` * `solver_noise_bound_gain_`
  float solver_noise_bound_gain_ = 1.0;
  float solver_noise_bound_      = voxel_size_ * solver_noise_bound_gain_;
  bool use_quatro_               = false;

  KISSMatcherConfig(const float voxel_size         = 0.3,
                    const float use_voxel_sampling = true,
                    const float use_quatro         = false,
                    const float thr_linearity      = 1.0,
                    const int num_max_corr         = 5000,
                    // Below params just works in general cases
                    const float normal_r_gain = 3.0,
                    const float fpfh_r_gain   = 5.0,
                    // The smaller, more conservative
                    const float robin_noise_bound_gain     = 1.0,
                    const float solver_noise_bound_gain    = 0.75,
                    const bool enable_noise_bound_clamping = true) {
    if (voxel_size < 5e-3) {
      throw std::runtime_error(
          "Too small voxel size has been given. Please check your voxel size.");
    }

    if (robin_noise_bound_gain < solver_noise_bound_gain) {
      throw std::runtime_error("`solver_noise_bound_gain` (" +
                               std::to_string(solver_noise_bound_gain) +
                               ") should be smaller than or equal to `robin_noise_bound_gain` (" +
                               std::to_string(robin_noise_bound_gain) + ").");
    }

    voxel_size_         = voxel_size;
    use_voxel_sampling_ = use_voxel_sampling;
    use_quatro_         = use_quatro;
    thr_linearity_      = thr_linearity;

    normal_radius_ = normal_r_gain * voxel_size;
    fpfh_radius_   = fpfh_r_gain * voxel_size;

    num_max_corr_            = num_max_corr;
    robin_noise_bound_gain_  = robin_noise_bound_gain;
    solver_noise_bound_gain_ = solver_noise_bound_gain;

    robin_noise_bound_  = voxel_size_ * robin_noise_bound_gain_;
    solver_noise_bound_ = voxel_size_ * solver_noise_bound_gain_;

    if ((robin_noise_bound_ > 1.0) && enable_noise_bound_clamping) {
      std::cout
          << "\033[1;33m[Warning] Too large `robin_noise_bound_` has been set.\n"
          << "Empirically, 1.0 tends to work better for large-scale maps.\n"
          << "If you do not want to clamp these values, disable `enable_noise_clamping`.\n\033[0m";
      robin_noise_bound_ = 1.0;
    }

    if ((solver_noise_bound_ > 1.0) && enable_noise_bound_clamping) {
      std::cout
          << "\033[1;33m[Warning] Too large `solver_noise bound_` has been set.\n"
          << "Empirically, 1.0 tends to work better for large-scale maps.\n"
          << "If you do not want to clamp these values, disable `enable_noise_clamping`.\n\033[0m";
      solver_noise_bound_ = 1.0;
    }
  }
};

class KISSMatcher {
 public:
  /**
   * @brief Constructor that initializes KISSMatcher with a voxel size.
   * @param voxel_size Size of the voxel grid used for setting other parameters.
   */
  explicit KISSMatcher(const float &voxel_size);

  /**
   * @brief Constructor that initializes KISSMatcher with a configuration object.
   * @param config Configuration parameters for the matcher.
   */
  explicit KISSMatcher(const KISSMatcherConfig &config);

  /**
   * @brief reset function
   */
  void reset();

  /**
   * @brief Resets the solver, used before pose estimation.
   * @note This function should call before pose estimation.
   */
  void resetSolver();

  /**
   * @brief Matches keypoints between source and target voxelized point clouds.
   * @param src Source point cloud.
   * @note Input clouds are automatically voxelized depending on `config_.use_voxel_sampling_`
   * @param tgt Target point cloud.
   * @return A pair of matched keypoints.
   */
  KeypointPair match(const std::vector<Eigen::Vector3f> &src,
                     const std::vector<Eigen::Vector3f> &tgt);

  /**
   * @brief Matches keypoints between source and target voxelized point clouds (Eigen format).
   * @param src Source point cloud in Eigen format.
   * @param tgt Target point cloud in Eigen format.
   * @return A pair of matched keypoints.
   */
  KeypointPair match(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src,
                     const Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt);

  /**
   * @brief Estimates the transformation between source and target point clouds.
   * @param src Source point cloud.
   * @param dst Target point cloud.
   * @return The estimated registration solution.
   */
  RegistrationSolution estimate(const std::vector<Eigen::Vector3f> &src,
                                const std::vector<Eigen::Vector3f> &dst);

  /**
   * @brief Retrieves input point clouds of FasterPFH.
   * @note Once, `config_.use_voxel_sampling_` is true, it outputs voxelized clouds
   * @return A pair of nput point clouds.
   */
  inline KeypointPair getProcessedInputClouds() {
    return {src_processed_, tgt_processed_};
  }

  /**
   * @brief Retrieves keypoints detected from FasterPFH.
   * @note The number of these keypoints is slightly smaller than 
   * or equal to the number of processed clouds.
   * @return A pair of keypoints from FasterPFH.
   */
  inline KeypointPair getKeypointsFromFasterPFH() {
    return {src_keypoints_, tgt_keypoints_};
  }

  /**
   * @brief Retrieves keypoints from the initial matching stage.
   * @note This function should be called after `match` function
   * @return A pair of initially matched keypoints.
   */
  inline KeypointPair getKeypointsFromInitialMatching() {
    return {src_matched_, tgt_matched_};
  }

  /**
   * @brief Retrieves the initial correspondences before refinement.
   * @return A list of initial correspondences (index pairs).
   */
  inline std::vector<std::pair<int, int>> getInitialCorrespondences() {
    return robin_matching_->getCrossCheckedCorrespondences();
  }

  /**
   * @brief Retrieves the final correspondences after refinement.
   * @return A list of final correspondences (index pairs).
   */
  inline std::vector<std::pair<int, int>> getFinalCorrespondences() {
    return robin_matching_->getFinalCorrespondences();
  }

  /**
   * @brief Gets the number of rotation inliers after solving.
   * @return Number of final rotation inliers from 
   * @graduated non-convexity (GNC) solver
   */
  inline size_t getNumRotationInliers(){
    return solver_->getRotationInliers().size();
  }

  /**
   * @brief Gets the number of final inliers after solving.
   * @return Number of final translation inliers from 
   * @component-wise translation estimation (COTE)
   * @note This number can be used to check whether the optimization is valid.
   */
  inline size_t getNumFinalInliers(){
    return solver_->getTranslationInliers().size();
  }

  void clear() {
    src_processed_.clear();
    tgt_processed_.clear();
    src_keypoints_.clear();
    tgt_keypoints_.clear();
    src_keypoints_.clear();
    tgt_keypoints_.clear();
    src_descriptors_.clear();
    tgt_descriptors_.clear();

    corr_.clear();

    processing_time_ = -1.0;
    extraction_time_ = -1.0;
    matching_time_   = -1.0;
    solver_time_     = -1.0;
  }

  double getProcessingTime();

  double getExtractionTime();

  double getRejectionTime();

  double getMatchingTime();

  double getSolverTime();

  void print();

 private:
  KISSMatcherConfig config_;

  std::unique_ptr<FasterPFH> faster_pfh_;
  std::unique_ptr<ROBINMatching> robin_matching_;
  std::unique_ptr<RobustRegistrationSolver> solver_;

  std::vector<Eigen::Vector3f> src_processed_;
  std::vector<Eigen::Vector3f> tgt_processed_;

  std::vector<Eigen::Vector3f> src_keypoints_;
  std::vector<Eigen::Vector3f> tgt_keypoints_;

  std::vector<Eigen::Vector3f> src_matched_;
  std::vector<Eigen::Vector3f> tgt_matched_;

  std::vector<Eigen::VectorXf> src_descriptors_;
  std::vector<Eigen::VectorXf> tgt_descriptors_;

  std::vector<std::pair<int, int>> corr_;

  // '-1' means that time has not been updated
  double processing_time_ = -1.0;
  double extraction_time_ = -1.0;
  double matching_time_   = -1.0;
  double solver_time_     = -1.0;
};

}  // namespace kiss_matcher
