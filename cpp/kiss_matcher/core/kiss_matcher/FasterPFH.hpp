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
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <kiss_matcher/tsl/robin_map.h>
#include <kiss_matcher/tsl/robin_set.h>

#include "kiss_matcher/kdtree/kdtree_tbb.hpp"
#include "kiss_matcher/points/point_cloud.hpp"

using MyKdTree = kiss_matcher::UnsafeKdTree<kiss_matcher::PointCloud>;

// was #define NOT_ASSIGNED -1 but results in a sign comparison warning
// as it is compared against the indices which are uint32_t or size_t
// This quiets the warning but is probably not what is intended.
// NEEDS thought!
#define NOT_ASSIGNED std::numeric_limits<uint32_t>::max()

#define UNASSIGNED_NORMAL                                  \
  Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), \
                  std::numeric_limits<float>::quiet_NaN(), \
                  std::numeric_limits<float>::quiet_NaN())

struct pair_hash {
  std::size_t operator()(const std::pair<uint32_t, uint32_t>& pair) const {
    return ((1 << 20) - 1) & (pair.first * 73856093 ^ pair.second * 19349669);
  }
};

namespace kiss_matcher {

struct FasterPFH {
  using Vector3fVector      = std::vector<Eigen::Vector3f>;
  using Vector3fVectorTuple = std::tuple<Vector3fVector, Vector3fVector>;
  using Voxel               = Eigen::Vector3i;
  struct Correspondences {
    std::vector<uint32_t> neighboring_indices;
    std::vector<float> neighboring_dists;
    bool is_planar = false;
    inline void clear() {
      if (!neighboring_indices.empty()) neighboring_indices.clear();
      if (!neighboring_dists.empty()) neighboring_dists.clear();
    }
  };

  FasterPFH() {}

  explicit FasterPFH(const float normal_radius,
                     const float fpfh_radius,
                     const float thr_linearity,
                     const std::string criteria            = "L2",
                     const bool use_non_maxima_suppression = false)
      : normal_radius_(normal_radius),
        fpfh_radius_(fpfh_radius),
        thr_linearity_(thr_linearity),
        criteria_(criteria),
        use_non_maxima_suppression_(use_non_maxima_suppression) {
    sqr_fpfh_radius_ = fpfh_radius * fpfh_radius;
  }

  inline void Clear() {
    corrs_fpfh_.clear();
    voxel_indices_.clear();
    spfh_indices_.clear();
    fpfh_indices_.clear();
    spfh_hist_lookup_.clear();  // why int? To employee `NOT_ASSIGNED`

    spfh_pairs_.clear();
    spfh_hash_table_.clear();

    num_points_per_voxel_.clear();
    num_valid_voxels_.clear();

    hist_f1_.clear();
    hist_f2_.clear();
    hist_f3_.clear();
  }

  void setInputCloud(const std::vector<Eigen::Vector3f>& points);

  //    void SetNormalsForValidPoints();

  //    void SetFPFHIndices();

  std::tuple<bool, Eigen::Vector3f> EstimateNormalVectorWithLinearityFiltering(
      const Correspondences& corr_fpfh,
      const float normal_radius,
      const float thr_linearity);
  bool IsNormalValid(const Eigen::Vector3f& normal);

  // For FPFH
  //    void Compute(const std::vector<Eigen::Vector3f> &points,
  //             std::vector<Eigen::Vector3f> &valid_points,
  //             std::vector<Eigen::VectorXf> &descriptors);

  void ComputeFeature(std::vector<Eigen::Vector3f>& points,
                      std::vector<Eigen::VectorXf>& descriptors);

  void ComputeSPFHSignatures(const tsl::robin_map<uint32_t, uint32_t>& spfh_hist_lookup,
                             std::vector<Eigen::VectorXf>& hist_f1,
                             std::vector<Eigen::VectorXf>& hist_f2,
                             std::vector<Eigen::VectorXf>& hist_f3);

  void ComputePointSPFHSignature(const uint32_t p_idx,
                                 Eigen::VectorXf& hist_f1,
                                 Eigen::VectorXf& hist_f2,
                                 Eigen::VectorXf& hist_f3);

  bool ComputePairFeatures(const Eigen::Vector3f& p1,
                           const Eigen::Vector3f& n1,
                           const Eigen::Vector3f& p2,
                           const Eigen::Vector3f& n2,
                           float& f1,
                           float& f2,
                           float& f3,
                           float& f4);

  void FilterIndicesCausingNaN(std::vector<uint32_t>& spfh_indices);

  void WeightPointSPFHSignature(const std::vector<Eigen::VectorXf>& hist_f1,
                                const std::vector<Eigen::VectorXf>& hist_f2,
                                const std::vector<Eigen::VectorXf>& hist_f3,
                                const std::vector<uint32_t>& indices,
                                const std::vector<double>& dists,
                                Eigen::VectorXf& fpfh_histogram);

  // For initialization
  float normal_radius_;
  float fpfh_radius_;
  float thr_linearity_;
  std::string criteria_;  // "L1" or "L2"
  bool use_non_maxima_suppression_ = false;

  float sqr_fpfh_radius_;
  int num_points_;
  int minimum_num_valid_ = 3;  // heuristics

  std::vector<int> num_points_per_voxel_;
  std::vector<int> num_valid_voxels_;

  std::vector<Voxel> indices_incremental_for_normal_;
  std::vector<Voxel> indices_incremental_for_fpfh_;

  // The sizes of below member variables are same with `num_points_`
  std::vector<Eigen::Vector3f> points_;
  std::vector<Eigen::Vector3f> normals_;
  std::vector<Correspondences> corrs_fpfh_;
  std::vector<Eigen::Vector3i> voxel_indices_;
  std::vector<bool> is_valid_;
  std::vector<bool> is_visited_;

  std::vector<uint32_t> spfh_indices_;  // voxels whose normals are valid
                                        //    tsl::robin_set<uint32_t> redundant_indices_;
  std::vector<uint32_t> fpfh_indices_;  // Originally, spfh_indices_ \ redundant_indices_

  tsl::robin_set<uint32_t> spfh_pairs_;
  tsl::robin_map<uint32_t, uint32_t> spfh_hist_lookup_;  // why int? To employee `NOT_ASSIGNED`
                                                         // From
  // https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/fpfh.h#L188
  /** \brief The number of subdivisions for each angular feature interval. */

  int nr_bins_f1_{11}, nr_bins_f2_{11}, nr_bins_f3_{11};

  tsl::robin_map<std::pair<uint32_t, uint32_t>, Eigen::Vector4f, pair_hash> spfh_hash_table_;

  /** \brief Placeholder for the f1 histogram. */
  std::vector<Eigen::VectorXf> hist_f1_;

  /** \brief Placeholder for the f2 histogram. */
  std::vector<Eigen::VectorXf> hist_f2_;

  /** \brief Placeholder for the f3 histogram. */
  std::vector<Eigen::VectorXf> hist_f3_;

  /** \brief Placeholder for a point's FPFH signature. */
  Eigen::VectorXf fpfh_histogram_;

  /** \brief Float constant = 1.0 / (2.0 * M_PI) */
  float d_pi_ = 1.0f / (2.0f * static_cast<float>(M_PI));
};
}  // namespace kiss_matcher
