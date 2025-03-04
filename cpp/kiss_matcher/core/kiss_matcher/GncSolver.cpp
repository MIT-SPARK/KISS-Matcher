/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include "kiss_matcher/GncSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kiss_matcher {
void ScalarTLSEstimator::estimate(const Eigen::RowVectorXd& X,
                                  const Eigen::RowVectorXd& ranges,
                                  double* estimate,
                                  Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  // check input parameters
  bool dimension_inconsistent = (X.rows() != ranges.rows()) || (X.cols() != ranges.cols());
  if (inliers) {
    dimension_inconsistent |= ((inliers->rows() != 1) || (inliers->cols() != ranges.cols()));
  }
  bool only_one_element = (X.rows() == 1) && (X.cols() == 1);
  assert(!dimension_inconsistent);
  assert(!only_one_element);  // TODO(jshi): admit a trivial solution

  size_t N = X.cols();
  std::vector<std::pair<double, int>> h;
  for (size_t i = 0; i < N; ++i) {
    h.push_back(std::make_pair(X(i) - ranges(i), i + 1));
    h.push_back(std::make_pair(X(i) + ranges(i), -i - 1));
  }

  // ascending order
  std::sort(h.begin(), h.end(), [](std::pair<double, int> a, std::pair<double, int> b) {
    return a.first < b.first;
  });

  // calculate weights
  Eigen::RowVectorXd weights = ranges.array().square();
  weights                    = weights.array().inverse();
  size_t nr_centers          = 2 * N;
  Eigen::RowVectorXd x_hat   = Eigen::MatrixXd::Zero(1, nr_centers);
  Eigen::RowVectorXd x_cost  = Eigen::MatrixXd::Zero(1, nr_centers);

  double ranges_inverse_sum    = ranges.sum();
  double dot_X_weights         = 0;
  double dot_weights_consensus = 0;
  int consensus_set_cardinal   = 0;
  double sum_xi                = 0;
  double sum_xi_square         = 0;

  for (size_t i = 0; i < nr_centers; ++i) {
    int idx     = static_cast<int>(std::abs(h.at(i).second)) - 1;  // Indices starting at 1
    int epsilon = (h.at(i).second > 0) ? 1 : -1;

    consensus_set_cardinal += epsilon;
    dot_weights_consensus += epsilon * weights(idx);
    dot_X_weights += epsilon * weights(idx) * X(idx);
    ranges_inverse_sum -= epsilon * ranges(idx);
    sum_xi += epsilon * X(idx);
    sum_xi_square += epsilon * X(idx) * X(idx);

    x_hat(i) = dot_X_weights / dot_weights_consensus;

    double residual =
        consensus_set_cardinal * x_hat(i) * x_hat(i) + sum_xi_square - 2 * sum_xi * x_hat(i);
    x_cost(i) = residual + ranges_inverse_sum;
  }

  size_t min_idx;
  x_cost.minCoeff(&min_idx);
  double estimate_temp = x_hat(min_idx);
  if (estimate) {
    // update estimate output if it's not nullptr
    *estimate = estimate_temp;
  }
  if (inliers) {
    // update inlier output if it's not nullptr
    *inliers = (X.array() - estimate_temp).array().abs() <= ranges.array();
  }
}

void ScalarTLSEstimator::estimate_tiled(const Eigen::RowVectorXd& X,
                                        const Eigen::RowVectorXd& ranges,
                                        const int& s,
                                        double* estimate,
                                        Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  // check input parameters
  bool dimension_inconsistent = (X.rows() != ranges.rows()) || (X.cols() != ranges.cols());
  if (inliers) {
    dimension_inconsistent |= ((inliers->rows() != 1) || (inliers->cols() != ranges.cols()));
  }
  bool only_one_element = (X.rows() == 1) && (X.cols() == 1);
  assert(!dimension_inconsistent);
  assert(!only_one_element);  // TODO(jshi): admit a trivial solution

  // Prepare variables for calculations
  int N = static_cast<int>(X.cols());
  Eigen::RowVectorXd h(N * 2);
  h << X - ranges, X + ranges;
  // ascending order
  std::sort(h.data(), h.data() + h.cols(), [](double a, double b) { return a < b; });
  // calculate interval centers
  Eigen::RowVectorXd h_centers = (h.head(h.cols() - 1) + h.tail(h.cols() - 1)) / 2;
  size_t nr_centers            = h_centers.cols();

  // calculate weights
  Eigen::RowVectorXd weights = ranges.array().square();
  weights                    = weights.array().inverse();

  Eigen::RowVectorXd x_hat  = Eigen::MatrixXd::Zero(1, nr_centers);
  Eigen::RowVectorXd x_cost = Eigen::MatrixXd::Zero(1, nr_centers);

  // loop tiling
  size_t ih_bound = ((nr_centers) & ~((s)-1));
  size_t jh_bound = ((N) & ~((s)-1));

  std::vector<double> ranges_inverse_sum_vec(nr_centers, 0);
  std::vector<double> dot_X_weights_vec(nr_centers, 0);
  std::vector<double> dot_weights_consensus_vec(nr_centers, 0);
  std::vector<std::vector<double>> X_consensus_table(nr_centers, std::vector<double>());

  auto inner_loop_f = [&](const size_t& i,
                          const size_t& jh,
                          const size_t& jl_lower_bound,
                          const size_t& jl_upper_bound) {
    double& ranges_inverse_sum           = ranges_inverse_sum_vec[i];
    double& dot_X_weights                = dot_X_weights_vec[i];
    double& dot_weights_consensus        = dot_weights_consensus_vec[i];
    std::vector<double>& X_consensus_vec = X_consensus_table[i];

    size_t j = 0;
    for (size_t jl = jl_lower_bound; jl < jl_upper_bound; ++jl) {
      j              = jh + jl;
      bool consensus = std::abs(X(j) - h_centers(i)) <= ranges(j);
      if (consensus) {
        dot_X_weights += X(j) * weights(j);
        dot_weights_consensus += weights(j);
        X_consensus_vec.push_back(X(j));
      } else {
        ranges_inverse_sum += ranges(j);
      }
    }

    if (static_cast<int>(j) == N - 1) {
      // x_hat(i) = dot(X(consensus), weights(consensus)) / dot(weights, consensus);
      x_hat(i) = dot_X_weights / dot_weights_consensus;

      // residual = X(consensus)-x_hat(i);
      Eigen::Map<Eigen::VectorXd> X_consensus(X_consensus_vec.data(), X_consensus_vec.size());
      Eigen::VectorXd residual = X_consensus.array() - x_hat(i);

      // x_cost(i) = dot(residual,residual) + sum(ranges(~consensus));
      x_cost(i) = residual.squaredNorm() + ranges_inverse_sum;
    }
  };

#pragma omp parallel for default(none) shared(jh_bound,                      \
                                                  ih_bound,                  \
                                                  ranges_inverse_sum_vec,    \
                                                  dot_X_weights_vec,         \
                                                  dot_weights_consensus_vec, \
                                                  X_consensus_table,         \
                                                  h_centers,                 \
                                                  weights,                   \
                                                  N,                         \
                                                  X,                         \
                                                  x_hat,                     \
                                                  x_cost,                    \
                                                  s,                         \
                                                  ranges,                    \
                                                  inner_loop_f)
  for (size_t ih = 0; ih < ih_bound; ih += s) {
    for (size_t jh = 0; jh < jh_bound; jh += s) {
      for (size_t il = 0; il < static_cast<size_t>(s); ++il) {
        size_t i = ih + il;
        inner_loop_f(i, jh, 0, s);
      }
    }
  }

  // finish the left over entries
  // 1. Finish the unfinished js
#pragma omp parallel for default(none) shared(jh_bound,                      \
                                                  ih_bound,                  \
                                                  ranges_inverse_sum_vec,    \
                                                  dot_X_weights_vec,         \
                                                  dot_weights_consensus_vec, \
                                                  X_consensus_table,         \
                                                  h_centers,                 \
                                                  weights,                   \
                                                  N,                         \
                                                  X,                         \
                                                  x_hat,                     \
                                                  x_cost,                    \
                                                  s,                         \
                                                  ranges,                    \
                                                  nr_centers,                \
                                                  inner_loop_f)
  for (size_t i = 0; i < nr_centers; ++i) {
    inner_loop_f(i, 0, jh_bound, N);
  }

  // 2. Finish the unfinished is
#pragma omp parallel for default(none) shared(jh_bound,                      \
                                                  ih_bound,                  \
                                                  ranges_inverse_sum_vec,    \
                                                  dot_X_weights_vec,         \
                                                  dot_weights_consensus_vec, \
                                                  X_consensus_table,         \
                                                  h_centers,                 \
                                                  weights,                   \
                                                  N,                         \
                                                  X,                         \
                                                  x_hat,                     \
                                                  x_cost,                    \
                                                  s,                         \
                                                  ranges,                    \
                                                  nr_centers,                \
                                                  inner_loop_f)
  for (size_t i = ih_bound; i < nr_centers; ++i) {
    inner_loop_f(i, 0, 0, N);
  }

  size_t min_idx;
  x_cost.minCoeff(&min_idx);
  double estimate_temp = x_hat(min_idx);
  if (estimate) {
    // update estimate output if it's not nullptr
    *estimate = estimate_temp;
  }
  if (inliers) {
    // update inlier output if it's not nullptr
    *inliers = (X.array() - estimate_temp).array().abs() <= ranges.array();
  }
}

void QuatroSolver::solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                                    const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                                    Eigen::Matrix3d* rotation,
                                    Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  assert(rotation);                  // make sure R is not a nullptr
  assert(src.cols() == dst.cols());  // check dimensions of input data
  assert(params_.gnc_factor > 1);    // make sure mu will increase
  assert(params_.noise_bound != 0);  // make sure noise sigma is not zero
  if (inliers) {
    assert(inliers->cols() == src.cols());
  }
  // Initialization
  *rotation = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, 2, Eigen::Dynamic> src_2d;
  Eigen::Matrix<double, 2, Eigen::Dynamic> dst_2d;
  // XY Coordinates for calculate yaw
  src_2d.resize(2, src.cols());
  dst_2d.resize(2, dst.cols());
  src_2d = src.topRows(2);
  dst_2d = dst.topRows(2);

  /**
   * Only the yaw rotation is estimated; thus, SO(2) estimation is performed
   */
  Eigen::Matrix2d rotation_2d = Eigen::Matrix2d::Identity();

  if (inliers) {
    assert(inliers->cols() == src.cols());
  }

  /**
   * Loop: terminate when:
   *    1. the change in cost in two consecutive runs is smaller than a user-defined threshold
   *    2. # iterations exceeds the maximum allowed
   *
   * Within each loop:
   * 1. fix weights and solve for R
   * 2. fix R and solve for weights
   */

  // Prepare some variables
  size_t match_size = src.cols();  // number of correspondences

  double mu = 1;  // arbitrary starting mu

  double prev_cost = std::numeric_limits<double>::infinity();
  cost_            = std::numeric_limits<double>::infinity();
  //  double noise_bound_sq = std::pow(params_.noise_bound, 2);
  static double rot_noise_bound = params_.noise_bound;
  static double noise_bound_sq  = std::pow(rot_noise_bound, 2);
  if (noise_bound_sq < 1e-16) {
    noise_bound_sq = 1e-2;
  }

  Eigen::Matrix<double, 2, Eigen::Dynamic> diffs(2, match_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> weights(1, match_size);
  weights.setOnes(1, match_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> residuals_sq(1, match_size);

  // Loop for performing GNC-TLS
  for (size_t i = 0; i < params_.max_iterations; ++i) {
    // Fix weights and perform SVD 2d rotation estimation
    rotation_2d = svdRot2d(src_2d, dst_2d, weights);

    // Calculate residuals squared
    diffs        = (dst_2d - rotation_2d * src_2d).array().square();
    residuals_sq = diffs.colwise().sum();
    if (i == 0) {
      // Initialize rule for mu
      double max_residual = residuals_sq.maxCoeff();
      mu                  = 1 / (2 * max_residual / noise_bound_sq - 1);
      // Degenerate case: mu = -1 because max_residual is very small
      // i.e., little to none noise
      if (mu <= 0) {
        // GNC-TLS terminated because maximum residual at initialization is very small
        break;
      }
    }

    // Fix R and solve for weights in closed form
    double th1 = (mu + 1) / mu * noise_bound_sq;
    double th2 = mu / (mu + 1) * noise_bound_sq;
    cost_      = 0;
    for (size_t j = 0; j < match_size; ++j) {
      // Also calculate cost in this loop
      // Note: the cost calculated is using the previously solved weights
      cost_ += weights(j) * residuals_sq(j);

      if (residuals_sq(j) >= th1) {
        weights(j) = 0;
      } else if (residuals_sq(j) <= th2) {
        weights(j) = 1;
      } else {
        weights(j) = sqrt(noise_bound_sq * mu * (mu + 1) / residuals_sq(j)) - mu;
        assert(weights(j) >= 0 && weights(j) <= 1);
      }
    }

    // Calculate cost
    double cost_diff = std::abs(cost_ - prev_cost);

    // Increase mu
    mu        = mu * params_.gnc_factor;
    prev_cost = cost_;

    if (cost_diff < params_.cost_threshold) {
      break;
    }
  }

  if (inliers) {
    for (size_t i = 0; i < static_cast<size_t>(weights.cols()); ++i) {
      (*inliers)(0, i) = weights(0, i) >= 0.4;
    }
  }

  /*
   * After the SO(2) estimation, the output matrix is filled by 'rotation_2d'
   */
  (*rotation).block<2, 2>(0, 0) = rotation_2d;
}

void TLSTranslationSolver::solveForTranslation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                                               const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                                               Eigen::Vector3d* translation,
                                               Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  assert(src.cols() == dst.cols());
  if (inliers) {
    assert(inliers->cols() == src.cols());
  }

  // Raw translation
  Eigen::Matrix<double, 3, Eigen::Dynamic> raw_translation = dst - src;

  // Error bounds for each measurements
  int N                                           = static_cast<int>(src.cols());
  double beta                                     = noise_bound_ * sqrt(cbar2_);
  Eigen::Matrix<double, 1, Eigen::Dynamic> alphas = beta * Eigen::MatrixXd::Ones(1, N);

  // Estimate x, y, and z component of translation: perform TLS on each row
  *inliers = Eigen::Matrix<bool, 1, Eigen::Dynamic>::Ones(1, N);
  Eigen::Matrix<bool, 1, Eigen::Dynamic> inliers_temp(1, N);
  for (size_t i = 0; i < static_cast<size_t>(raw_translation.rows()); ++i) {
    auto& translation_ref = (*translation)(i);
    tls_estimator_.estimate(raw_translation.row(i), alphas, &translation_ref, &inliers_temp);
    // element-wise AND using component-wise product (Eigen 3.2 compatible)
    // a point is an inlier iff. x,y,z are all inliers
    *inliers = (*inliers).cwiseProduct(inliers_temp);
  }
}

RobustRegistrationSolver::RobustRegistrationSolver(const RobustRegistrationSolver::Params& params) {
  reset(params);
}

Eigen::Matrix<double, 3, Eigen::Dynamic> RobustRegistrationSolver::computeTIMs(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& v,
    Eigen::Matrix<int, 2, Eigen::Dynamic>* map) {
  size_t N = v.cols();
  Eigen::Matrix<double, 3, Eigen::Dynamic> vtilde(3, N * (N - 1) / 2);
  map->resize(2, N * (N - 1) / 2);

#pragma omp parallel for default(none) shared(N, v, vtilde, map)
  for (size_t i = 0; i < N - 1; i++) {
    // Calculate some important indices
    // For each measurement, we compute the TIMs between itself and all the measurements after it.
    // For example:
    // i=0: add N-1 TIMs
    // i=1: add N-2 TIMs
    // etc..
    // i=k: add N-1-k TIMs
    // And by arithmatic series, we can get the starting index of each segment be:
    // k*N - k*(k+1)/2
    size_t segment_start_idx = i * N - i * (i + 1) / 2;
    size_t segment_cols      = N - 1 - i;

    // calculate TIM
    Eigen::Matrix<double, 3, 1> m                 = v.col(i);
    Eigen::Matrix<double, 3, Eigen::Dynamic> temp = v - m * Eigen::MatrixXd::Ones(1, N);

    // concatenate to the end of the tilde vector
    vtilde.middleCols(segment_start_idx, segment_cols) = temp.rightCols(segment_cols);

    // populate the index map
    Eigen::Matrix<int, 2, Eigen::Dynamic> map_addition(2, N);
    for (size_t j = 0; j < N; ++j) {
      map_addition(0, j) = static_cast<int>(i);
      map_addition(1, j) = static_cast<int>(j);
    }
    map->middleCols(segment_start_idx, segment_cols) = map_addition.rightCols(segment_cols);
  }

  return vtilde;
}

RegistrationSolution RobustRegistrationSolver::solve(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst) {
  assert(rotation_solver_ && translation_solver_);

  size_t num_corr = src.cols();
  indices_.reserve(num_corr);
  for (size_t i = 0; i < num_corr; ++i) {
    indices_.push_back(i);
  }

  // chain graph
  pruned_src_tims_.resize(3, num_corr);
  pruned_dst_tims_.resize(3, num_corr);
  for (size_t i = 0; i < num_corr; ++i) {
    const auto& root = indices_[i];
    size_t leaf;
    if (i != num_corr - 1) {
      leaf = indices_[i + 1];
    } else {
      leaf = indices_[0];
    }
    pruned_src_tims_.col(i) = src.col(leaf) - src.col(root);
    pruned_dst_tims_.col(i) = dst.col(leaf) - dst.col(root);
  }

  // NOTE(hlim): Actually, we don't need to do multiplying by 2, which is redundant,
  // but to follow the original TEASER++ code's convention, we preserve this part.
  // Instead of removing this block, please adjust the noise bound gain of the solver.
  auto params = rotation_solver_->getParams();
  params.noise_bound *= 2;

  rotation_solver_->setParams(params);

  // Solve for rotation
  solveForRotation(pruned_src_tims_, pruned_dst_tims_);

  // Save indices of inlier TIMs from GNC rotation estimation
  for (size_t i = 0; i < static_cast<size_t>(rotation_inliers_mask_.cols()); ++i) {
    if (rotation_inliers_mask_[i]) {
      rotation_inliers_.emplace_back(i);
    }
  }

  if (rotation_inliers_.size() < 2) {
    return RegistrationSolution();
  }

  // Solve for translation
  solveForTranslation(solution_.rotation * src, dst);

  // Find the final inliers
  translation_inliers_.clear();
  for (size_t i = 0; i < static_cast<size_t>(translation_inliers_mask_.cols()); ++i) {
    if (translation_inliers_mask_(i)) {
      translation_inliers_.push_back(i);
    }
  }

  // Update validity flag
  if (translation_inliers_.empty()) {
    solution_.valid = false;
  } else {
    solution_.valid = true;
  }

  return solution_;
}

Eigen::Vector3d RobustRegistrationSolver::solveForTranslation(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& v1,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& v2) {
  translation_inliers_mask_.resize(1, v1.cols());
  translation_solver_->solveForTranslation(
      v1, v2, &(solution_.translation), &translation_inliers_mask_);
  return solution_.translation;
}

Eigen::Matrix3d RobustRegistrationSolver::solveForRotation(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& v1,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& v2) {
  rotation_inliers_mask_.resize(1, v1.cols());
  rotation_solver_->solveForRotation(v1, v2, &(solution_.rotation), &rotation_inliers_mask_);
  return solution_.rotation;
}

void GNCTLSRotationSolver::solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                                            Eigen::Matrix3d* rotation,
                                            Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  assert(rotation);                  // make sure R is not a nullptr
  assert(src.cols() == dst.cols());  // check dimensions of input data
  assert(params_.gnc_factor > 1);    // make sure mu will increase
  assert(params_.noise_bound != 0);  // make sure noise sigma is not zero
  if (inliers) {
    assert(inliers->cols() == src.cols());
  }

  /**
   * Loop: terminate when:
   *    1. the change in cost in two consecutive runs is smaller than a user-defined threshold
   *    2. # iterations exceeds the maximum allowed
   *
   * Within each loop:
   * 1. fix weights and solve for R
   * 2. fix R and solve for weights
   */

  // Prepare some variables
  size_t match_size = src.cols();  // number of correspondences

  double mu = 1;  // arbitrary starting mu

  double prev_cost      = std::numeric_limits<double>::infinity();
  cost_                 = std::numeric_limits<double>::infinity();
  double noise_bound_sq = std::pow(params_.noise_bound, 2);
  if (noise_bound_sq < 1e-16) {
    noise_bound_sq = 1e-2;
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> diffs(3, match_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> weights(1, match_size);
  weights.setOnes(1, match_size);
  Eigen::Matrix<double, 1, Eigen::Dynamic> residuals_sq(1, match_size);

  // Loop for performing GNC-TLS
  for (size_t i = 0; i < params_.max_iterations; ++i) {
    // Fix weights and perform SVD rotation estimation
    *rotation = svdRot(src, dst, weights);

    // Calculate residuals squared
    diffs        = (dst - (*rotation) * src).array().square();
    residuals_sq = diffs.colwise().sum();
    if (i == 0) {
      // Initialize rule for mu
      double max_residual = residuals_sq.maxCoeff();
      mu                  = 1 / (2 * max_residual / noise_bound_sq - 1);
      // Degenerate case: mu = -1 because max_residual is very small
      // i.e., little to none noise
      if (mu <= 0) {
        // "GNC-TLS terminated because maximum residual at initialization is very small.
        break;
      }
    }

    // Fix R and solve for weights in closed form
    double th1 = (mu + 1) / mu * noise_bound_sq;
    double th2 = mu / (mu + 1) * noise_bound_sq;
    cost_      = 0;
    for (size_t j = 0; j < match_size; ++j) {
      // Also calculate cost in this loop
      // Note: the cost calculated is using the previously solved weights
      cost_ += weights(j) * residuals_sq(j);

      if (residuals_sq(j) >= th1) {
        weights(j) = 0;
      } else if (residuals_sq(j) <= th2) {
        weights(j) = 1;
      } else {
        weights(j) = sqrt(noise_bound_sq * mu * (mu + 1) / residuals_sq(j)) - mu;
        assert(weights(j) >= 0 && weights(j) <= 1);
      }
    }

    // Calculate cost
    double cost_diff = std::abs(cost_ - prev_cost);

    // Increase mu
    mu        = mu * params_.gnc_factor;
    prev_cost = cost_;

    if (cost_diff < params_.cost_threshold) {
      break;
    }
  }

  if (inliers) {
    for (size_t i = 0; i < match_size; ++i) {
      (*inliers)(0, i) = weights(0, i) >= 0.5;
    }
  }
}
}  // namespace kiss_matcher
