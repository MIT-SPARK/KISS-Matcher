/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */
#pragma once

#include <chrono>
#include <execution>
#include <fstream>
#include <mutex>
#include <numeric>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <flann/flann.hpp>
#include <robin/robin.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#define USE_UNORDERED_MAP 1

namespace kiss_matcher {

class ROBINMatching {
 public:
  typedef std::vector<Eigen::VectorXf> Feature;
  typedef flann::Index<flann::L2<float>> KDTree;

  ROBINMatching() {}

  ROBINMatching(const float noise_bound,
                const int num_max_corr  = 5000,
                const float tuple_scale = 0.95);

  // Warning: Do not use `use_ratio_test` in the scan-level registration,
  // because setting `use_ratio_test` to `true` sometimes reduces the number of correspondences
  std::vector<std::pair<int, int>> establishCorrespondences(

      std::vector<Eigen::Vector3f>& source_points,
      std::vector<Eigen::Vector3f>& target_points,
      Feature& source_features,
      Feature& target_features,
      std::string robin_mode,
      float tuple_scale   = 0.95,
      bool use_ratio_test = false);

  // For a deeper understanding, please refer to Section III.D
  // ttps://arxiv.org/pdf/2409.15615
  std::vector<size_t> applyOutlierPruning(const std::vector<Eigen::Vector3f>& src_matched,
                                          const std::vector<Eigen::Vector3f>& tgt_matched,
                                          const std::string& robin_mode = "max_core");

  inline std::vector<std::pair<int, int>> getCrossCheckedCorrespondences() {
    std::vector<std::pair<int, int>> corres_out;
    corres_out.reserve(corres_cross_checked_.size());
    for (const auto& corres_tuple : corres_cross_checked_) {
      if (swapped_) {
        corres_out.emplace_back(std::pair<int, int>(corres_tuple.second, corres_tuple.first));
      } else {
        corres_out.emplace_back(std::pair<int, int>(corres_tuple.first, corres_tuple.second));
      }
    }
    return corres_out;
  }

  inline std::vector<std::pair<int, int>> getCrosscheckedCorrespondences() {
    return corres_cross_checked_;
  }

  inline std::vector<std::pair<int, int>> getFinalCorrespondences() { return corres_; }

  double getRejectionTime() { return rejection_time_; }

  size_t getNumInitialCorrespondences() { return num_init_corr_; }

  size_t getNumPrunedCorrespondences() { return num_pruned_corr_; }

 private:
  size_t fi_ = 0;  // source idx
  size_t fj_ = 1;  // destination idx

  ssize_t nPti_;
  ssize_t nPtj_;

  bool swapped_ = false;

  template <typename T>
  void buildKDTree(const std::vector<T>& data, KDTree* tree);

  template <typename T>
  void searchKDTree(const KDTree& tree,
                    const T& input,
                    std::array<int, 2>& indices,
                    std::array<float, 2>& dists,
                    int nn);

  template <typename T>
  void searchKDTreeAll(KDTree* tree,
                       const std::vector<T>& inputs,
                       std::vector<int>& indices,
                       std::vector<float>& dists,
                       int nn);

  // NOTE(hlim): Without `isValidIndex, sometimes segmentation fault occurs.
  // For this reason, we over-add isValidIndex for the safety purpose
  bool isValidIndex(const size_t index, const size_t vector_size) { return index < vector_size; }

  void match(const std::string& robin_mode, float tuple_scale, bool use_ratio_test = false);

  void setStatuses();

  void runTupleTest(const std::vector<std::pair<int, int>>& corres,
                    std::vector<std::pair<int, int>>& corres_out,
                    const float tuple_scale);

  // For a deeper understanding, please refer to Section III.D
  // ttps://arxiv.org/pdf/2409.15615
  void applyOutlierPruning(const std::vector<std::pair<int, int>>& corres,
                           std::vector<std::pair<int, int>>& corres_out,
                           const std::string& robin_mode = "max_core");

  std::vector<std::pair<int, int>> corres_cross_checked_;
  std::vector<std::pair<int, int>> corres_;
  std::vector<std::vector<Eigen::Vector3f>> pointcloud_;
  std::vector<Feature> features_;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      means_;  // for normalization

  size_t num_max_corr_;
  size_t num_init_corr_   = 0;
  size_t num_pruned_corr_ = 0;

  float noise_bound_;

  float tuple_test_ratio_ = 0.95;

  float thr_dist_       = 30;   // Empirically, potentially imprecise matching is rejected
  float thr_ratio_test_ = 0.9;  // The lower, the more strict
  float sqr_thr_dist_   = thr_dist_ * thr_dist_;

  double rejection_time_;
};

}  // namespace kiss_matcher
