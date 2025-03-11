/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "kiss_matcher/ROBINMatching.hpp"

#include <algorithm>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kiss_matcher {

ROBINMatching::ROBINMatching(const float noise_bound,
                             const int num_max_corr,
                             const float tuple_scale) {
  noise_bound_      = noise_bound;
  num_max_corr_     = num_max_corr;
  tuple_test_ratio_ = tuple_scale;
}

// NOTE(hlim): I don't recommend you using `use_ratio_test` in most cases,
// because setting `use_ratio_test` to `true` sometimes significantly reduces the number of
// correspondences
std::vector<std::pair<int, int>> ROBINMatching::establishCorrespondences(
    std::vector<Eigen::Vector3f>& source_points,
    std::vector<Eigen::Vector3f>& target_points,
    Feature& source_features,
    Feature& target_features,
    std::string robin_mode,
    float tuple_scale,
    bool use_ratio_test) {
  pointcloud_.clear();
  features_.clear();

  corres_cross_checked_.clear();
  corres_.clear();

  Feature cloud_features;
  pointcloud_.emplace_back(source_points);
  pointcloud_.emplace_back(target_points);

  features_.emplace_back(source_features);
  features_.emplace_back(target_features);

  setStatuses();
  match(robin_mode, tuple_scale, use_ratio_test);

  return corres_;
}

void ROBINMatching::match(const std::string& robin_mode, float tuple_scale, bool use_ratio_test) {
  KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
  buildKDTree(features_[fi_], &feature_tree_i);

  KDTree feature_tree_j(flann::KDTreeSingleIndexParams(15));
  buildKDTree(features_[fj_], &feature_tree_j);

  // NOTE(hlim): `2` indicates that we save the two distances between the two closest descriptors.
  int num_candidates = use_ratio_test ? 2 : 1;
  std::vector<std::array<int, 2>> corres_K(nPtj_, {0,0});
  std::vector<std::array<int, 2>> corres_K2(nPti_, {0,0});
  std::vector<std::array<float, 2>> dis_j(nPtj_, {0.0, 0.0});
  std::vector<std::array<float, 2>> dis_i(nPti_, {0.0, 0.0});

  std::vector<std::pair<int, int>> corres_ij;
  std::vector<std::pair<int, int>> corres_ji;

  std::vector<int> i_to_j_multi_flann(nPti_, -1);
  std::vector<int> j_to_i_multi_flann(nPtj_, -1);

  corres_cross_checked_.clear();

  std::vector<std::tuple<int, int, float>> matched_pairs;  // (ji, j, ratio)

  tbb::parallel_for(tbb::blocked_range<size_t>(0, nPtj_), [&](tbb::blocked_range<size_t> r) {
    for (size_t j = r.begin(); j < r.end(); ++j) {
      searchKDTree(feature_tree_i, features_[fj_][j], corres_K[j], dis_j[j], num_candidates);
      bool is_over_ratio = use_ratio_test ? dis_j[j][0] > thr_ratio_test_ * dis_j[j][1] : false;
      if (dis_j[j][0] > sqr_thr_dist_ || is_over_ratio) {
        continue;
      }
      if (corres_K[j][0] >= 0 && corres_K[j][0] < nPti_) {
        if (i_to_j_multi_flann[corres_K[j][0]] == -1) {
          searchKDTree(feature_tree_j,
                       features_[fi_][corres_K[j][0]],
                       corres_K2[corres_K[j][0]],
                       dis_i[corres_K[j][0]],
                       1);
          i_to_j_multi_flann[corres_K[j][0]] = corres_K2[corres_K[j][0]][0];
        }
        j_to_i_multi_flann[j] = corres_K[j][0];
      }
    }
  });

  // Note(hlim): ratio-based filtering was better than distance-based filtering!
  // Success rate in the KITTI 10m benchmark:
  // float ratio = dis_j[j][0] / dis_j[j][1]; <- 98.56%
  // float ratio = dis_j[j][0];               <- 97.84%
  matched_pairs.reserve(nPti_);
  for (ssize_t j = 0; j < nPtj_; j++) {
    int ji = j_to_i_multi_flann[j];
    if (ji != -1 && j == i_to_j_multi_flann[ji]) {
      float ratio = use_ratio_test ? dis_j[j][0] / dis_j[j][1] : 0.0;
      matched_pairs.emplace_back(ji, j, ratio);
    }
  }

  if (matched_pairs.size() > num_max_corr_) {
    if (use_ratio_test) {
      std::sort(matched_pairs.begin(), matched_pairs.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
      });
      matched_pairs.resize(num_max_corr_);
    } else {
      // Fisher-Yates-like partial shuffle
      // NOTE(hlim) Due to the randomness, sometimes it fails
      std::random_device rd;
      std::mt19937 gen(rd());

      for (size_t i = 0; i < num_max_corr_; ++i) {
        size_t j = i + gen() % (matched_pairs.size() - i);
        std::swap(matched_pairs[i], matched_pairs[j]);
      }
      matched_pairs.resize(num_max_corr_);
    }
  }

  for (const auto& corres : matched_pairs) {
    corres_cross_checked_.emplace_back(
        std::pair<int, int>(std::get<0>(corres), std::get<1>(corres)));
  }

  // Compatibility test for outlier pruning
  corres_.clear();
  auto t_rejection_init = std::chrono::high_resolution_clock::now();
  if (robin_mode == "None") {
    runTupleTest(corres_cross_checked_, corres_, tuple_scale);
  } else if (robin_mode == "max_core" || robin_mode == "max_clique") {
    applyOutlierPruning(corres_cross_checked_, corres_, robin_mode);
  } else {
    std::invalid_argument("Wrong ROBIN mode has come.");
  }
  auto t_rejection_end = std::chrono::high_resolution_clock::now();
  rejection_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_rejection_end - t_rejection_init)
          .count();
}

void ROBINMatching::setStatuses() {
  fi_ = 0;  // source idx
  fj_ = 1;  // destination idx

  swapped_ = false;

  if (pointcloud_[fj_].size() > pointcloud_[fi_].size()) {
    size_t temp = fi_;
    fi_         = fj_;
    fj_         = temp;
    swapped_    = true;
  }

  nPti_ = pointcloud_[fi_].size();
  nPtj_ = pointcloud_[fj_].size();
}

void ROBINMatching::runTupleTest(const std::vector<std::pair<int, int>>& corres,
                                 std::vector<std::pair<int, int>>& corres_out,
                                 const float /*tuple_scale*/) {
  if (!corres.empty()) {
    size_t rand0, rand1, rand2;
    size_t idi0, idi1, idi2;
    size_t idj0, idj1, idj2;
    size_t ncorr           = corres.size();
    size_t number_of_trial = ncorr * 100;
    std::vector<std::pair<int, int>> corres_tuple;
    corres_tuple.reserve(ncorr);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> distribution(0, ncorr - 1);

    std::vector<bool> is_already_included(ncorr, false);

    corres_out.clear();

    for (size_t i = 0; i < number_of_trial; i++) {
      rand0 = distribution(gen);
      rand1 = distribution(gen);
      rand2 = distribution(gen);

      idi0 = corres[rand0].first;
      idj0 = corres[rand0].second;
      idi1 = corres[rand1].first;
      idj1 = corres[rand1].second;
      idi2 = corres[rand2].first;
      idj2 = corres[rand2].second;

      if (!isValidIndex(idi0, nPti_)) {
        continue;
      }
      if (!isValidIndex(idi1, nPti_)) {
        continue;
      }
      if (!isValidIndex(idi2, nPti_)) {
        continue;
      }
      if (!isValidIndex(idj0, nPtj_)) {
        continue;
      }
      if (!isValidIndex(idj1, nPtj_)) {
        continue;
      }
      if (!isValidIndex(idj2, nPtj_)) {
        continue;
      }

      // collect 3 points from i-th fragment
      const Eigen::Vector3f& pti0 = pointcloud_[fi_][idi0];
      const Eigen::Vector3f& pti1 = pointcloud_[fi_][idi1];
      const Eigen::Vector3f& pti2 = pointcloud_[fi_][idi2];

      float li0 = (pti0 - pti1).norm();
      float li1 = (pti1 - pti2).norm();
      float li2 = (pti2 - pti0).norm();

      // collect 3 points from j-th fragment
      const Eigen::Vector3f& ptj0 = pointcloud_[fj_][idj0];
      const Eigen::Vector3f& ptj1 = pointcloud_[fj_][idj1];
      const Eigen::Vector3f& ptj2 = pointcloud_[fj_][idj2];

      float lj0 = (ptj0 - ptj1).norm();
      float lj1 = (ptj1 - ptj2).norm();
      float lj2 = (ptj2 - ptj0).norm();

      static float thr = noise_bound_;
      if (li0 - thr > lj0 || lj0 > li0 + thr) {
        continue;
      }
      if (li1 - thr > lj1 || lj1 > li1 + thr) {
        continue;
      }
      if (li2 - thr > lj2 || lj2 > li2 + thr) {
        continue;
      }

      if (!is_already_included[rand0]) {
        corres_tuple.emplace_back(idi0, idj0);
        is_already_included[rand0] = true;
      }
      if (!is_already_included[rand1]) {
        corres_tuple.emplace_back(idi1, idj1);
        is_already_included[rand1] = true;
      }
      if (!is_already_included[rand2]) {
        corres_tuple.emplace_back(idi2, idj2);
        is_already_included[rand2] = true;
      }
      //      }
    }
    corres_out.clear();

    for (size_t i = 0; i < corres_tuple.size(); ++i) {
      if (swapped_) {
        corres_out.emplace_back(std::pair<int, int>(corres_tuple[i].second, corres_tuple[i].first));
      } else {
        corres_out.emplace_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
      }
    }
  } else {
    std::invalid_argument("Wrong tuple constraint has come.");
  }
}

void ROBINMatching::applyOutlierPruning(const std::vector<std::pair<int, int>>& corres,
                                        std::vector<std::pair<int, int>>& corres_out,
                                        const std::string& robin_mode) {
  if (!corres.empty()) {
    size_t ncorr = corres.size();
    std::vector<bool> is_already_included(ncorr, false);
    corres_out.clear();

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_robin(3, ncorr);
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_robin(3, ncorr);

#pragma omp parallel for
    for (size_t i = 0; i < ncorr; ++i) {
      src_robin.col(i) = pointcloud_[fi_][corres[i].first].cast<double>();
      tgt_robin.col(i) = pointcloud_[fj_][corres[i].second].cast<double>();
    }

    auto* g = robin::Make3dRegInvGraph(src_robin, tgt_robin, noise_bound_);

    const auto& filtered_indices = [&]() {
      // NOTE(hlim): Just use max core mode.
      // `max_clique` not only took more time but also showed slightly worse performance.
      if (robin_mode == "max_core") {
        return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CORE);
      } else if (robin_mode == "max_clique") {
        return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CLIQUE);
      } else {
        throw std::runtime_error("Something's wrong!");
      }
    }();

    for (size_t i = 0; i < filtered_indices.size(); ++i) {
      const auto& corres_pair = corres[filtered_indices[i]];
      if (swapped_) {
        corres_out.emplace_back(corres_pair.second, corres_pair.first);
      } else {
        corres_out.emplace_back(corres_pair.first, corres_pair.second);
      }
    }

    num_init_corr_   = ncorr;
    num_pruned_corr_ = filtered_indices.size();
  } else {
    std::invalid_argument("Wrong tuple constraint has come.");
  }
}

std::vector<size_t> ROBINMatching::applyOutlierPruning(
    const std::vector<Eigen::Vector3f>& src_matched,
    const std::vector<Eigen::Vector3f>& tgt_matched,
    const std::string& robin_mode) {
  if (src_matched.size() != tgt_matched.size()) {
    std::runtime_error("The size of `src_matched` and `tgt_matched` should be same.");
  }
  if (src_matched.size() < 2 || tgt_matched.size() < 2) {
    std::runtime_error("Too few matched points are given.");
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_robin(3, src_matched.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_robin(3, tgt_matched.size());

  num_init_corr_ = src_matched.size();
#pragma omp parallel for
  for (size_t i = 0; i < num_init_corr_; ++i) {
    src_robin.col(i) = src_matched[i].cast<double>();
    tgt_robin.col(i) = tgt_matched[i].cast<double>();
  }

  auto* g = robin::Make3dRegInvGraph(src_robin, tgt_robin, noise_bound_);

  const auto& filtered_indices = [&]() {
    // NOTE(hlim): Just use max core mode.
    // `max_clique` not only took more time but also showed slightly worse performance.
    if (robin_mode == "max_core") {
      return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CORE);
    } else if (robin_mode == "max_clique") {
      return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CLIQUE);
    } else {
      throw std::runtime_error("Something's wrong!");
    }
  }();

  num_pruned_corr_ = filtered_indices.size();
  return filtered_indices;
}

template <typename T>
void ROBINMatching::buildKDTree(const std::vector<T>& data, ROBINMatching::KDTree* tree) {
  int rows, dim;
  rows = static_cast<int>(data.size());
  dim  = static_cast<int>(data[0].size());
  std::vector<float> dataset(rows * dim);
  flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < dim; j++) dataset[i * dim + j] = data[i][j];
  KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
  temp_tree.buildIndex();
  *tree = temp_tree;
}

template <typename T>
void ROBINMatching::searchKDTree(const KDTree& tree,
                                 const T& input,
                                 std::array<int, 2>& indices,
                                 std::array<float, 2>& dists,
                                 int nn) {
  size_t rows_t = 1;
  size_t dim    = input.size();

  std::vector<float> query;
  query.resize(rows_t * dim);
  for (size_t i = 0; i < dim; ++i) query[i] = input(i);
  flann::Matrix<float> query_mat(&query[0], rows_t, dim);

  flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
  flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

  static auto flann_params = flann::SearchParams(128);
  flann_params.cores       = 1;
  tree.knnSearch(query_mat, indices_mat, dists_mat, nn, flann_params);
}

template <typename T>
void ROBINMatching::searchKDTreeAll(KDTree* tree,
                                    const std::vector<T>& inputs,
                                    std::vector<int>& indices,
                                    std::vector<float>& dists,
                                    int nn) {
  size_t dim = inputs[0].size();

  std::vector<float> query(inputs.size() * dim);
  for (size_t i = 0; i < inputs.size(); ++i) {
    for (int j = 0; j < dim; ++j) {
      query[i * dim + j] = inputs[i](j);
    }
  }
  flann::Matrix<float> query_mat(&query[0], inputs.size(), dim);

  indices.resize(inputs.size() * nn);
  dists.resize(inputs.size() * nn);
  flann::Matrix<int> indices_mat(&indices[0], inputs.size(), nn);
  flann::Matrix<float> dists_mat(&dists[0], inputs.size(), nn);

  auto flann_params  = flann::SearchParams(128);
  flann_params.cores = 8;
  tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann_params);
}

}  // namespace kiss_matcher
