// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <kiss_matcher/points/traits.hpp>

namespace kiss_matcher {
namespace traits {

template <>
struct Traits<Eigen::MatrixXd> {
  static size_t size(const Eigen::MatrixXd& points) { return points.rows(); }
  static bool has_points(const Eigen::MatrixXd& points) { return points.rows(); }
  static Eigen::Vector4d point(const Eigen::MatrixXd& points, size_t i) {
    return Eigen::Vector4d(points(i, 0), points(i, 1), points(i, 2), 1.0);
  }
};

}  // namespace traits
}  // namespace kiss_matcher
