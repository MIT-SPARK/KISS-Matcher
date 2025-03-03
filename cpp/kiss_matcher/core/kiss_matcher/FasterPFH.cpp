/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "kiss_matcher/FasterPFH.hpp"

#include <algorithm>
#include <execution>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_reduce.h>

namespace kiss_matcher {
void CheckNaNandBreak(const std::vector<Eigen::VectorXf> &vecs) {
  for (const auto &vec : vecs) {
    if (vec.array().isNaN().any()) {
      throw std::runtime_error("Vector contains NaN.");
    }
  }
}

// Normal whose values are all nan are considered invalid
// Please refer to `UNASSIGNED_NORMAL`
bool FasterPFH::IsNormalValid(const Eigen::Vector3f &normal) {
  if (std::isnan(normal(0))) return false;
  if (std::isnan(normal(1))) return false;
  if (std::isnan(normal(2))) return false;

  // Otherwise, it's valid!
  return true;
}

std::tuple<bool, Eigen::Vector3f> FasterPFH::EstimateNormalVectorWithLinearityFiltering(
    const Correspondences &corr_fpfh,
    const float normal_radius,
    const float thr_linearity) {
  float thr_radius;
  if (criteria_ == "L1") {
    thr_radius = normal_radius;
  } else if (criteria_ == "L2") {
    thr_radius = normal_radius * normal_radius;
  } else {
    throw std::runtime_error("Wrong criteria given");
  }

  Eigen::Vector3f mean       = Eigen::Vector3f::Zero();
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  int count                  = 0;

  for (std::size_t j = 0; j < corr_fpfh.neighboring_indices.size(); ++j) {
    if (corr_fpfh.neighboring_dists[j] < thr_radius) {
      int idx                      = corr_fpfh.neighboring_indices[j];
      const Eigen::Vector3f &point = points_[idx];
      mean += point;
      ++count;
    }
  }

  Eigen::Vector3f normal;
  if (count < minimum_num_valid_) {
    normal(0) = std::numeric_limits<double>::quiet_NaN();
    normal(1) = std::numeric_limits<double>::quiet_NaN();
    normal(2) = std::numeric_limits<double>::quiet_NaN();
    return std::make_tuple(false, normal);
  }
  mean /= static_cast<float>(count);

  // Calculate covariance
  for (std::size_t j = 0; j < corr_fpfh.neighboring_indices.size(); ++j) {
    if (corr_fpfh.neighboring_dists[j] < thr_radius) {
      Eigen::Vector3f point     = points_[corr_fpfh.neighboring_indices[j]];
      Eigen::Vector3f deviation = point - mean;
      covariance += deviation * deviation.transpose();
    }
  }
  covariance /= static_cast<float>(count - 1);

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(covariance, Eigen::DecompositionOptions::ComputeFullU);
  const Eigen::Vector3f singular_values = svd.singularValues();

  float linearity = (singular_values(0) - singular_values(1)) / singular_values(0);
  if (linearity > thr_linearity) {
    normal(0) = std::numeric_limits<double>::quiet_NaN();
    normal(1) = std::numeric_limits<double>::quiet_NaN();
    normal(2) = std::numeric_limits<double>::quiet_NaN();

    return std::make_tuple(false, normal);
  }

  normal = (svd.matrixU().col(2));
  // For consistent re-orientation of normals
  if (-mean.dot(normal) < 0) {
    normal = normal * -1.0f;
  }

  return std::make_tuple(true, normal);
}

void FasterPFH::setInputCloud(const std::vector<Eigen::Vector3f> &points) {
  Clear();

  points_     = points;
  num_points_ = static_cast<int>(points.size());
  normals_.resize(num_points_, UNASSIGNED_NORMAL);
  corrs_fpfh_.resize(num_points_);
  num_valid_voxels_.resize(num_points_, 0);
  is_valid_.resize(num_points_, false);
  is_visited_.resize(num_points_, false);
}

// https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/impl/fpfh.hpp#L270
void FasterPFH::ComputeFeature(std::vector<Eigen::Vector3f> &points,
                               std::vector<Eigen::VectorXf> &descriptors) {
  // `spfh_indices`: indices of points that have valid normals
  //  it is used when calculating FPFH descriptor
  spfh_indices_.clear();
  std::vector<uint32_t> empty_vector;
  empty_vector.reserve(num_points_);
  spfh_indices_.reserve(num_points_);

  kiss_matcher::PointCloud cloud_nano;
  cloud_nano.points.resize(points_.size());
  for (size_t i = 0; i < points_.size(); i++) {
    const auto &p        = points_[i];
    cloud_nano.points[i] = Eigen::Vector4d(p(0), p(1), p(2), 1.0);
  }
  MyKdTree kdtree(cloud_nano);

  // auto t_s_n    = std::chrono::high_resolution_clock::now();
  spfh_indices_ = tbb::parallel_reduce(
      // Range
      tbb::blocked_range<uint32_t>(0, num_points_),
      // Identity
      empty_vector,
      // 1st lambda: Parallel computation
      [&](const tbb::blocked_range<uint32_t> &r,
          std::vector<uint32_t> local_indices) -> std::vector<uint32_t> {
        local_indices.reserve(r.size());
        for (uint32_t i = r.begin(); i != r.end(); ++i) {
          if (criteria_ == "L2") {
            // Then, neighboring_dists are squared distances
            std::vector<std::pair<uint32_t, double> > indices_dists;
            indices_dists.reserve(1000);
            // NOTE: squared distance is used, and outputs are also squared values
            size_t num_results =
                kdtree.radius_search(cloud_nano.point(i), sqr_fpfh_radius_, indices_dists);
            if (num_results > 0) {
              for (const auto &[idx, sqr_dist] : indices_dists) {
                corrs_fpfh_[i].neighboring_indices.push_back(idx);
                corrs_fpfh_[i].neighboring_dists.push_back(sqr_dist);
              }
            }
          }

          if (corrs_fpfh_[i].neighboring_indices.size() > 2) {
            const auto &[is_valid, normal] = EstimateNormalVectorWithLinearityFiltering(
                corrs_fpfh_[i], normal_radius_, thr_linearity_);
            is_valid_[i] = is_valid;
            normals_[i]  = normal;
          }

          if (is_valid_[i]) {
            local_indices.push_back(i);
          }
        }
        return local_indices;
      },
      // 2nd lambda: Parallel reduction
      [](std::vector<uint32_t> a, const std::vector<uint32_t> &b) -> std::vector<uint32_t> {
        a.insert(a.end(),  //
                 std::make_move_iterator(b.begin()),
                 std::make_move_iterator(b.end()));
        return a;
      });

  // Important!
  // Without this function, the final descriptors have NaN values
  FilterIndicesCausingNaN(spfh_indices_);
  // auto t_e_n = std::chrono::high_resolution_clock::now();

  // Setting up the SPFH histogram bins and lookup table
  spfh_hist_lookup_.clear();

  // Initialize the arrays that will store the SPFH signatures
  std::size_t data_size = spfh_indices_.size();
  hist_f1_.clear();
  hist_f2_.clear();
  hist_f3_.clear();

  hist_f1_.reserve(data_size);
  hist_f2_.reserve(data_size);
  hist_f3_.reserve(data_size);

  static Eigen::VectorXf bin_f1 = Eigen::VectorXf::Zero(nr_bins_f1_);
  static Eigen::VectorXf bin_f2 = Eigen::VectorXf::Zero(nr_bins_f2_);
  static Eigen::VectorXf bin_f3 = Eigen::VectorXf::Zero(nr_bins_f3_);

  std::uint32_t tmp_i = 0;
  for (const auto &p_idx : spfh_indices_) {
    spfh_hist_lookup_[p_idx] = tmp_i;
    ++tmp_i;

    hist_f1_.emplace_back(bin_f1);
    hist_f2_.emplace_back(bin_f2);
    hist_f3_.emplace_back(bin_f3);
  }

  // Compute SPFH signatures
  ComputeSPFHSignatures(spfh_hist_lookup_, hist_f1_, hist_f2_, hist_f3_);

  // Currently, we assume that spfh_indices_ == fpfh_indices_
  // auto t_e_s    = std::chrono::high_resolution_clock::now();
  fpfh_indices_ = spfh_indices_;
  size_t N      = fpfh_indices_.size();
  points.resize(N);
  descriptors.resize(N);
  // Iterate over the entire index vector

  tbb::parallel_for(tbb::blocked_range<size_t>(0, N), [&](const tbb::blocked_range<size_t> &r) {
    //    tbb::parallel_for(0, N, [&](const int& j) {
    for (size_t j = r.begin(); j != r.end(); ++j) {
      const int p_idx = fpfh_indices_[j];
      std::vector<uint32_t> nn_indices;
      std::vector<double> nn_dists;
      nn_indices.reserve(corrs_fpfh_[p_idx].neighboring_indices.size());
      nn_dists.reserve(corrs_fpfh_[p_idx].neighboring_dists.size());
      //      nn_cardinalities.resize(p_voxel.valid_neighboring_voxels.size());

      const auto &indices = corrs_fpfh_[p_idx].neighboring_indices;
      const auto &dists   = corrs_fpfh_[p_idx].neighboring_dists;

      for (size_t i = 0; i < indices.size(); ++i) {
        if (is_valid_[indices[i]]) {
          nn_indices.emplace_back(spfh_hist_lookup_[indices[i]]);
          nn_dists.emplace_back(dists[i]);
        }
      }

      points[j] = points_[p_idx];
      WeightPointSPFHSignature(hist_f1_, hist_f2_, hist_f3_, nn_indices, nn_dists, descriptors[j]);
    }
  });

  // auto t_e_w = std::chrono::high_resolution_clock::now();
  //  std::cout << "[Normal]: "
  //            << std::chrono::duration_cast<std::chrono::microseconds>(t_e_n - t_s_n).count() /
  //               1000000.0 << " sec" << std::endl;
  //  std::cout << "[SPFH]: "
  //            << std::chrono::duration_cast<std::chrono::microseconds>(t_e_s - t_e_n).count() /
  //               1000000.0 << " sec" << std::endl;
  //  std::cout << "[WSPFH]: "
  //            << std::chrono::duration_cast<std::chrono::microseconds>(t_e_w - t_e_s).count() /
  //               1000000.0 << " sec" << std::endl;
}
//
void FasterPFH::ComputeSPFHSignatures(const tsl::robin_map<uint32_t, uint32_t> &spfh_hist_lookup,
                                      std::vector<Eigen::VectorXf> &hist_f1,
                                      std::vector<Eigen::VectorXf> &hist_f2,
                                      std::vector<Eigen::VectorXf> &hist_f3) {
  // Compute SPFH signatures for every point that needs them
  tbb::parallel_for_each(
      spfh_hist_lookup.cbegin(), spfh_hist_lookup.cend(), [&](const auto &lookup) {
        const auto &p_idx = lookup.first;
        const auto &i     = lookup.second;

        ComputePointSPFHSignature(p_idx, hist_f1[i], hist_f2[i], hist_f3[i]);
      });
}
//
//// From
/// https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/impl/fpfh.hpp#L64
void FasterPFH::ComputePointSPFHSignature(const uint32_t p_idx,
                                          Eigen::VectorXf &hist_f1,
                                          Eigen::VectorXf &hist_f2,
                                          Eigen::VectorXf &hist_f3) {
  Eigen::Vector4f pfh_tuple = Eigen::Vector4f::Zero();

  int num_all_valid_points = 0;
  for (const auto &neighbor_idx : corrs_fpfh_[p_idx].neighboring_indices) {
    if (is_valid_[neighbor_idx] && neighbor_idx != p_idx) {
      ++num_all_valid_points;
    }
  }

  // Factorization constant
  float hist_incr = 100.0f / static_cast<float>(num_all_valid_points);

  for (const auto &neighbor_idx : corrs_fpfh_[p_idx].neighboring_indices) {
    if (!is_valid_[neighbor_idx] && neighbor_idx == p_idx) {
      continue;
    }

    if (!ComputePairFeatures(points_[p_idx],
                             normals_[p_idx],
                             points_[neighbor_idx],
                             normals_[neighbor_idx],
                             pfh_tuple[0],
                             pfh_tuple[1],
                             pfh_tuple[2],
                             pfh_tuple[3]))
      continue;

    // Normalize the f1, f2, f3 features and push them in the histogram
    int h_index = static_cast<int>(std::floor(nr_bins_f1_ * ((pfh_tuple[0] + M_PI) * d_pi_)));
    if (h_index < 0) h_index = 0;
    if (h_index >= nr_bins_f1_) h_index = nr_bins_f1_ - 1;
    hist_f1(h_index) += hist_incr;

    h_index = static_cast<int>(std::floor(nr_bins_f2_ * ((pfh_tuple[1] + 1.0) * 0.5)));
    if (h_index < 0) h_index = 0;
    if (h_index >= nr_bins_f2_) h_index = nr_bins_f2_ - 1;
    hist_f2(h_index) += hist_incr;

    h_index = static_cast<int>(std::floor(nr_bins_f3_ * ((pfh_tuple[2] + 1.0) * 0.5)));
    if (h_index < 0) h_index = 0;
    if (h_index >= nr_bins_f3_) h_index = nr_bins_f3_ - 1;
    hist_f3(h_index) += hist_incr;
  }
}
//
//// From https://github.com/PointCloudLibrary/pcl/blob/master/features/src/pfh.cpp#L45
bool FasterPFH::ComputePairFeatures(const Eigen::Vector3f &p1,
                                    const Eigen::Vector3f &n1,
                                    const Eigen::Vector3f &p2,
                                    const Eigen::Vector3f &n2,
                                    float &f1,
                                    float &f2,
                                    float &f3,
                                    float &f4) {
  Eigen::Vector3f dp2p1 = p2 - p1;
  f4                    = dp2p1.norm();

  // Below line means that p1 == p2
  if (f4 == 0.0f) {
    f1 = f2 = f3 = f4 = 0.0f;
    return (false);
  }

  Eigen::Vector3f n1_copy = n1, n2_copy = n2;
  float angle1 = static_cast<float>(n1_copy.dot(dp2p1) / f4);

  // Make sure the same point is selected as 1 and 2 for each pair
  float angle2 = static_cast<float>(n2_copy.dot(dp2p1) / f4);
  if (std::acos(std::fabs(angle1)) > std::acos(std::fabs(angle2))) {
    // switch p1 and p2
    n1_copy = n2;
    n2_copy = n1;
    dp2p1 *= (-1);
    f3 = -angle2;
  } else {
    f3 = angle1;
  }

  // Create a Darboux frame coordinate system u-v-w
  // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
  Eigen::Vector3f v = dp2p1.cross(n1_copy);
  float v_norm      = static_cast<float>(v.norm());
  if (v_norm == 0.0f) {
    f1 = f2 = f3 = f4 = 0.0f;
    return (false);
  }
  // Normalize v
  v /= v_norm;

  Eigen::Vector3f w = n1_copy.cross(v);
  // Do not have to normalize w - it is a unit vector by construction

  //  v[3] = 0.0f;
  f2 = static_cast<float>(v.dot(n2_copy));
  //  w[3] = 0.0f;
  // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
  f1 = static_cast<float>(std::atan2(w.dot(n2_copy), n1_copy.dot(n2_copy)));  //

  return (true);
}

void FasterPFH::FilterIndicesCausingNaN(std::vector<uint32_t> &spfh_indices) {
  size_t i = 0;
  while (i < spfh_indices.size()) {
    int target_idx          = spfh_indices[i];
    int num_valid_neighbors = 0;
    for (const auto &idx : corrs_fpfh_[target_idx].neighboring_indices) {
      if (is_valid_[idx]) {
        ++num_valid_neighbors;
      }
    }

    if (num_valid_neighbors < minimum_num_valid_) {
      std::iter_swap(spfh_indices.begin() + i, spfh_indices.end() - 1);
      spfh_indices.pop_back();
    } else {
      ++i;
    }
  }
}

void FasterPFH::WeightPointSPFHSignature(const std::vector<Eigen::VectorXf> &hist_f1,
                                         const std::vector<Eigen::VectorXf> &hist_f2,
                                         const std::vector<Eigen::VectorXf> &hist_f3,
                                         const std::vector<uint32_t> &indices,
                                         const std::vector<double> &dists,
                                         Eigen::VectorXf &fpfh_histogram) {
  assert(indices.size() == dists.size());
  double sum_f1 = 0.0, sum_f2 = 0.0, sum_f3 = 0.0;
  float weight = 0.0, val_f1, val_f2, val_f3;

  // Get the number of bins from the histograms size
  const auto nr_bins_f1  = hist_f1[0].size();
  const auto nr_bins_f2  = hist_f2[0].size();
  const auto nr_bins_f3  = hist_f3[0].size();
  const auto nr_bins_f12 = nr_bins_f1 + nr_bins_f2;

  // Clear the histogram
  fpfh_histogram.setZero(nr_bins_f1 + nr_bins_f2 + nr_bins_f3);

  // Use the entire patch
  for (std::size_t idx = 0; idx < indices.size(); ++idx) {
    if (dists[idx] == 0) {
      weight = 1.0f;
      // Weight the SPFH of the query point with the SPFH of its neighbors
      for (Eigen::MatrixXf::Index f1_i = 0; f1_i < nr_bins_f1; ++f1_i) {
        val_f1 = hist_f1[indices[idx]](f1_i) * weight;
        sum_f1 += val_f1;
        fpfh_histogram[f1_i] += val_f1;
      }

      for (Eigen::MatrixXf::Index f2_i = 0; f2_i < nr_bins_f2; ++f2_i) {
        val_f2 = hist_f2[indices[idx]](f2_i) * weight;
        sum_f2 += val_f2;
        fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
      }

      for (Eigen::MatrixXf::Index f3_i = 0; f3_i < nr_bins_f3; ++f3_i) {
        val_f3 = hist_f3[indices[idx]](f3_i) * weight;
        sum_f3 += val_f3;
        fpfh_histogram[f3_i + nr_bins_f12] += val_f3;
      }
    }
    // Minus the query point itself
    if (dists[idx] == 0 || indices[idx] == NOT_ASSIGNED) continue;

    // Standard weighting function used
    // HT: note that squared distance showed better performance
    weight = 1.0f / dists[idx];

    // Weight the SPFH of the query point with the SPFH of its neighbors
    for (Eigen::MatrixXf::Index f1_i = 0; f1_i < nr_bins_f1; ++f1_i) {
      val_f1 = hist_f1[indices[idx]](f1_i) * weight;
      sum_f1 += val_f1;
      fpfh_histogram[f1_i] += val_f1;
    }

    for (Eigen::MatrixXf::Index f2_i = 0; f2_i < nr_bins_f2; ++f2_i) {
      val_f2 = hist_f2[indices[idx]](f2_i) * weight;
      sum_f2 += val_f2;
      fpfh_histogram[f2_i + nr_bins_f1] += val_f2;
    }

    for (Eigen::MatrixXf::Index f3_i = 0; f3_i < nr_bins_f3; ++f3_i) {
      val_f3 = hist_f3[indices[idx]](f3_i) * weight;
      sum_f3 += val_f3;
      fpfh_histogram[f3_i + nr_bins_f12] += val_f3;
    }
  }

  if (sum_f1 != 0) sum_f1 = 100.0 / sum_f1;  // histogram values sum up to 100
  if (sum_f2 != 0) sum_f2 = 100.0 / sum_f2;  // histogram values sum up to 100
  if (sum_f3 != 0) sum_f3 = 100.0 / sum_f3;  // histogram values sum up to 100

  // Adjust final FPFH values
  const auto denormalize_with = [](auto factor) {
    return [=](const auto &data) { return data * factor; };
  };

  auto last = fpfh_histogram.data();
  last      = std::transform(last, last + nr_bins_f1, last, denormalize_with(sum_f1));
  last      = std::transform(last, last + nr_bins_f2, last, denormalize_with(sum_f2));
  std::transform(last, last + nr_bins_f3, last, denormalize_with(sum_f3));
}

}  // namespace kiss_matcher
