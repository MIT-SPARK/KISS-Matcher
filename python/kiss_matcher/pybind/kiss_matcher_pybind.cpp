// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "kiss_matcher/GncSolver.hpp"
#include "kiss_matcher/KISSMatcher.hpp"

#include "./stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;
using namespace kiss_matcher;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

PYBIND11_MODULE(kiss_matcher, m) {
  m.doc()               = "Pybind11 bindings for KISSMatcher library";
  m.attr("__version__") = "0.3.1";

  // auto vector3fvector = pybind_eigen_vector_of_vector<Eigen::Vector3f>(
  //     m,
  //     "_Vector3fVector",
  //     "std::vector<Eigen::Vector3f>",
  //     py::py_array_to_vectors_float<Eigen::Vector3f>);

  // auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
  //     m,
  //     "_Vector3dVector",
  //     "std::vector<Eigen::Vector3d>",
  //     py::py_array_to_vectors_double<Eigen::Vector3d>);

  // Bind KISSMatcherConfig
  // py::class_<KISSMatcherConfig>(m, "KISSMatcherConfig")
  //     .def(py::init<float, bool, float, int, float, float, float, float>(),
  //          "voxel_size"_a              = 0.3,
  //          "use_quatro"_a              = false,
  //          "thr_linearity"_a           = 1.0,
  //          "num_max_corr"_a            = 5000,
  //          "normal_r_gain"_a           = 3.0,
  //          "fpfh_r_gain"_a             = 5.0,
  //          "robin_noise_bound_gain"_a  = 1.0,
  //          "solver_noise_bound_gain"_a = 0.75)
  //     .def_readwrite("voxel_size", &KISSMatcherConfig::voxel_size_)
  //     .def_readwrite("normal_radius", &KISSMatcherConfig::normal_radius_)
  //     .def_readwrite("fpfh_radius", &KISSMatcherConfig::fpfh_radius_)
  //     .def_readwrite("thr_linearity", &KISSMatcherConfig::thr_linearity_)
  //     .def_readwrite("robin_noise_bound_gain", &KISSMatcherConfig::robin_noise_bound_gain_)
  //     .def_readwrite("solver_noise_bound_gain", &KISSMatcherConfig::solver_noise_bound_gain_)
  //     .def_readwrite("num_max_corr", &KISSMatcherConfig::num_max_corr_)
  //     .def_readwrite("use_quatro", &KISSMatcherConfig::use_quatro_);

  py::class_<KISSMatcherConfig>(m, "KISSMatcherConfig")
      .def(py::init<float, bool, bool, float, int, float, float, float, float, bool>(),
           "voxel_size"_a                  = 0.3,
           "use_voxel_sampling"_a          = true,
           "use_quatro"_a                  = false,
           "thr_linearity"_a               = 1.0,
           "num_max_corr"_a                = 5000,
           "normal_r_gain"_a               = 3.0,
           "fpfh_r_gain"_a                 = 5.0,
           "robin_noise_bound_gain"_a      = 1.0,
           "solver_noise_bound_gain"_a     = 0.75,
           "enable_noise_bound_clamping"_a = true)
      .def_readwrite("voxel_size", &KISSMatcherConfig::voxel_size_)
      .def_readwrite("use_voxel_sampling", &KISSMatcherConfig::use_voxel_sampling_)
      .def_readwrite("use_quatro", &KISSMatcherConfig::use_quatro_)
      .def_readwrite("thr_linearity", &KISSMatcherConfig::thr_linearity_)
      .def_readwrite("num_max_corr", &KISSMatcherConfig::num_max_corr_)
      .def_readwrite("normal_radius", &KISSMatcherConfig::normal_radius_)
      .def_readwrite("fpfh_radius", &KISSMatcherConfig::fpfh_radius_)
      .def_readwrite("robin_noise_bound_gain", &KISSMatcherConfig::robin_noise_bound_gain_)
      .def_readwrite("solver_noise_bound_gain", &KISSMatcherConfig::solver_noise_bound_gain_)
      .def_readwrite("robin_noise_bound", &KISSMatcherConfig::robin_noise_bound_)
      .def_readwrite("solver_noise_bound", &KISSMatcherConfig::solver_noise_bound_);

  // Bind RegistrationSolution
  py::class_<RegistrationSolution>(m, "RegistrationSolution")
      .def_readwrite("valid", &RegistrationSolution::valid)
      .def_readwrite("translation", &RegistrationSolution::translation)
      .def_readwrite("rotation", &RegistrationSolution::rotation);

  // Bind KISSMatcher
  py::class_<KISSMatcher>(m, "KISSMatcher")
      .def(py::init<const float &>(), "voxel_size"_a)
      .def(py::init<const KISSMatcherConfig &>(), "config"_a)
      .def("reset", &KISSMatcher::reset, "Reset the matcher state")
      .def("reset_solver", &KISSMatcher::resetSolver, "Reset the solver")
      .def("match",
           py::overload_cast<const std::vector<Eigen::Vector3f> &,
                             const std::vector<Eigen::Vector3f> &>(&KISSMatcher::match),
           "src_voxelized"_a,
           "tgt_voxelized"_a,
           "Match keypoints from source and target")
      .def("match",
           py::overload_cast<const Eigen::Matrix<double, 3, Eigen::Dynamic> &,
                             const Eigen::Matrix<double, 3, Eigen::Dynamic> &>(&KISSMatcher::match),
           "src_voxelized"_a,
           "tgt_voxelized"_a,
           "Match keypoints from Eigen matrices")
      .def("estimate", &KISSMatcher::estimate, "src"_a, "dst"_a, "Estimate transformation")
      .def("get_keypoints_from_faster_pfh",
           &KISSMatcher::getKeypointsFromFasterPFH,
           "Get keypoints from FasterPFH")
      .def("get_keypoints_from_initial_matching",
           &KISSMatcher::getKeypointsFromInitialMatching,
           "Get keypoints from initial matching")
      .def("get_initial_correspondences",
           &KISSMatcher::getInitialCorrespondences,
           "Get initial correspondences")
      .def("get_final_correspondences",
           &KISSMatcher::getFinalCorrespondences,
           "Get final correspondences")
      .def("clear", &KISSMatcher::clear, "Clear internal states")
      .def("get_extraction_time", &KISSMatcher::getExtractionTime, "Get feature extraction time")
      .def("get_rejection_time", &KISSMatcher::getRejectionTime, "Get outlier rejection time")
      .def("get_matching_time", &KISSMatcher::getMatchingTime, "Get matching time")
      .def("get_solver_time", &KISSMatcher::getSolverTime, "Get solver time")
      .def("print", &KISSMatcher::print, "Print matcher state");
}
