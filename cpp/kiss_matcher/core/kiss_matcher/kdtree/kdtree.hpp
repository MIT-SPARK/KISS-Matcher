// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <memory>

#include <Eigen/Core>
#include <kiss_matcher/kdtree/nanoflann.hpp>
#include <kiss_matcher/kdtree/traits.hpp>
#include <kiss_matcher/points/traits.hpp>

namespace kiss_matcher {

/// @brief Unsafe KdTree with arbitrary nanoflann Adaptor.
/// @note  This class does not hold the ownership of the input points.
///        You must keep the input points along with this class.
template <class PointCloud, template <typename, typename, int, typename> class Adaptor>
class UnsafeKdTreeGeneric {
 public:
  using Ptr       = std::shared_ptr<UnsafeKdTreeGeneric>;
  using ConstPtr  = std::shared_ptr<const UnsafeKdTreeGeneric>;
  using ThisClass = UnsafeKdTreeGeneric<PointCloud, Adaptor>;
  using Index =
      Adaptor<kiss_matcher::L2_Simple_Adaptor<double, ThisClass, double>, ThisClass, 3, uint32_t>;

  /// @brief Constructor
  /// @param points  Input points
  explicit UnsafeKdTreeGeneric(const PointCloud &points)
      : points(points), index(3, *this, kiss_matcher::KDTreeSingleIndexAdaptorParams(10)) {
    index.buildIndex();
  }

  /// @brief Constructor
  /// @param points  Input points
  /// @params num_threads  Number of threads used for building the index (This argument is only
  /// valid for OMP implementation)
  explicit UnsafeKdTreeGeneric(const PointCloud &points, int num_threads)
      : points(points), index(3, *this, kiss_matcher::KDTreeSingleIndexAdaptorParams(10)) {
    index.buildIndex(num_threads);
  }

  ~UnsafeKdTreeGeneric() {}

  // Interfaces for nanoflann
  size_t kdtree_get_point_count() const { return traits::size(points); }
  double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return traits::point(points, idx)[dim];
  }

  template <class BBox>
  bool kdtree_get_bbox(BBox &) const {
    return false;
  }

  /// @brief Find k-nearest neighbors
  size_t knn_search(const Eigen::Vector4d &pt,
                    size_t k,
                    size_t *k_indices,
                    double *k_sq_dists) const {
    return index.knnSearch(pt.data(), k, k_indices, k_sq_dists);
  }

  /// @brief Find neighbors within the radius
  size_t radius_search(const Eigen::Vector4d &pt,
                       double r,
                       std::vector<std::pair<uint32_t, double>> &indices_sq_dists) const {
    kiss_matcher::SearchParams params;
    params.sorted = false;  // to boost the speed
    return index.radiusSearch(pt.data(), r, indices_sq_dists, params);
  }

 private:
  const PointCloud &points;  ///< Input points
  Index index;               ///< KdTree index
};

/// @brief KdTree  with arbitrary nanoflann Adaptor
template <class PointCloud, template <typename, typename, int, typename> class Adaptor>
class KdTreeGeneric {
 public:
  using Ptr      = std::shared_ptr<KdTreeGeneric>;
  using ConstPtr = std::shared_ptr<const KdTreeGeneric>;

  /// @brief Constructor
  /// @param points  Input points
  explicit KdTreeGeneric(const std::shared_ptr<const PointCloud> &points)
      : points(points), tree(*points) {}

  /// @brief Constructor
  /// @param points  Input points
  explicit KdTreeGeneric(const std::shared_ptr<const PointCloud> &points, int num_threads)
      : points(points), tree(*points, num_threads) {}

  ~KdTreeGeneric() {}

  /// @brief Find k-nearest neighbors
  size_t knn_search(const Eigen::Vector4d &pt,
                    size_t k,
                    size_t *k_indices,
                    double *k_sq_dists) const {
    return tree.knn_search(pt, k, k_indices, k_sq_dists);
  }

  /// @brief Find neighbors within the radius
  size_t radius_search(const Eigen::Vector4d &pt,
                       double r,
                       std::vector<std::pair<uint32_t, double>> &indices_sq_dists) const {
    kiss_matcher::SearchParams params;
    params.sorted = false;  // to boost the speed
    return tree.radius_search(pt.data(), r, indices_sq_dists, params);
  }

 private:
  const std::shared_ptr<const PointCloud> points;       ///< Input points
  const UnsafeKdTreeGeneric<PointCloud, Adaptor> tree;  ///< KdTree
};

/// @brief Standard KdTree (unsafe)
template <class PointCloud>
using UnsafeKdTree = UnsafeKdTreeGeneric<PointCloud, kiss_matcher::KDTreeSingleIndexAdaptor>;

/// @brief Standard KdTree
template <class PointCloud>
using KdTree = KdTreeGeneric<PointCloud, kiss_matcher::KDTreeSingleIndexAdaptor>;

namespace traits {

template <class PointCloud, template <typename, typename, int, typename> class Adaptor>
struct Traits<UnsafeKdTreeGeneric<PointCloud, Adaptor>> {
  static size_t knn_search(const UnsafeKdTreeGeneric<PointCloud, Adaptor> &tree,
                           const Eigen::Vector4d &point,
                           size_t k,
                           size_t *k_indices,
                           double *k_sq_dists) {
    return tree.knn_search(point, k, k_indices, k_sq_dists);
  }

  static size_t radius_search(const UnsafeKdTreeGeneric<PointCloud, Adaptor> &tree,
                              const Eigen::Vector4d &point,
                              double r,
                              std::vector<std::pair<uint32_t, double>> &indices_sq_dists) {
    kiss_matcher::SearchParams params;
    params.sorted = false;  // to boost the speed
    return tree.radius_search(point, r, indices_sq_dists);
  }
};

template <class PointCloud, template <typename, typename, int, typename> class Adaptor>
struct Traits<KdTreeGeneric<PointCloud, Adaptor>> {
  static size_t knn_search(const KdTreeGeneric<PointCloud, Adaptor> &tree,
                           const Eigen::Vector4d &point,
                           size_t k,
                           size_t *k_indices,
                           double *k_sq_dists) {
    return tree.knn_search(point, k, k_indices, k_sq_dists);
  }

  static size_t radius_search(const KdTreeGeneric<PointCloud, Adaptor> &tree,
                              const Eigen::Vector4d &point,
                              double r,
                              std::vector<std::pair<uint32_t, double>> &indices_sq_dists) {
    kiss_matcher::SearchParams params;
    params.sorted = false;  // to boost the speed
    return tree.radius_search(point, r, indices_sq_dists);
  }
};

}  // namespace traits

}  // namespace kiss_matcher
