#include <filesystem>

#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include "quatro/quatro_utils.h"
void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {
  int N = pc.points.size();

  pc_colored.clear();
  pcl::PointXYZRGB pt_tmp;
  for (int i = 0; i < N; ++i) {
    const auto &pt = pc.points[i];
    pt_tmp.x       = pt.x;
    pt_tmp.y       = pt.y;
    pt_tmp.z       = pt.z;
    pt_tmp.r       = color[0];
    pt_tmp.g       = color[1];
    pt_tmp.b       = color[2];
    pc_colored.points.emplace_back(pt_tmp);
  }
}

std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <src_pcd_file> <tgt_pcd_file> <resolution> <yaw_aug_angle> <roll_aug_angle>" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl(new pcl::PointCloud<pcl::PointXYZ>);

  const std::string src_path = argv[1];
  const std::string tgt_path = argv[2];
  const float resolution     = std::stof(argv[3]);

  Eigen::Matrix4f yaw_transform = Eigen::Matrix4f::Identity();
  if (argc > 4) {
    float yaw_aug_angle = std::stof(argv[4]);             // Yaw angle in degrees
    float yaw_rad       = yaw_aug_angle * M_PI / 180.0f;  // Convert to radians

    yaw_transform(0, 0) = std::cos(yaw_rad);
    yaw_transform(0, 1) = -std::sin(yaw_rad);
    yaw_transform(1, 0) = std::sin(yaw_rad);
    yaw_transform(1, 1) = std::cos(yaw_rad);
  }

  Eigen::Matrix4f roll_transform = Eigen::Matrix4f::Identity();
  if (argc > 5) {
    float roll_aug_angle = std::stof(argv[5]);             // Yaw angle in degrees
    float roll_rad       = roll_aug_angle * M_PI / 180.0f;  // Convert to radians

    roll_transform(1, 1) = std::cos(roll_rad);
    roll_transform(1, 2) = -std::sin(roll_rad);
    roll_transform(2, 1) = std::sin(roll_rad);
    roll_transform(2, 2) = std::cos(roll_rad);
  }

  ssize_t stress = 0;
  if (argc > 6) {
    stress = std::stoi(argv[6]);  // Number of stress iterations
  }

  Eigen::Matrix4f rotation_transform = yaw_transform * roll_transform;

  std::cout << "Source input: " << src_path << "\n";
  std::cout << "Target input: " << tgt_path << "\n";
  int src_load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(src_path, *src_pcl);
  int tgt_load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(tgt_path, *tgt_pcl);

  std::vector<int> src_indices;
  std::vector<int> tgt_indices;
  pcl::removeNaNFromPointCloud(*src_pcl, *src_pcl, src_indices);
  pcl::removeNaNFromPointCloud(*tgt_pcl, *tgt_pcl, tgt_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*src_pcl, *rotated_src_pcl, rotation_transform);
  src_pcl = rotated_src_pcl;

  const auto& src_vec = convertCloudToVec(*src_pcl);
  const auto& tgt_vec = convertCloudToVec(*tgt_pcl);

  std::cout << "\033[1;32mLoad complete!\033[0m\n";

  kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig(resolution);
  // NOTE(hlim): Two important parameters for enhancing performance
  // 1. `config.use_quatro_ (default: false)`
  // If the rotation is predominantly around the yaw axis, set `use_quatro_` to true like:
  // config.use_quatro_ = true;
  // Otherwise, the default mode activates SO(3)-based GNC.
  // E.g., in the case of `VBR-Collosseo`, it should be set as `false`.
  //
  // 2. `config.use_ratio_test_ (default: true)`
  // If dealing with a scan-level point cloud, the impact of `use_ratio_test_` is insignificant.
  // Plus, setting `use_ratio_test_` to false helps speed up slightly
  // If you want to try your own scan at a scan-level or loop closing situation,
  // setting `false` boosts the inference speed.
  // config.use_ratio_test_ = false;

  kiss_matcher::KISSMatcher matcher(config);
  kiss_matcher::RegistrationSolution solution;
  solution = matcher.estimate(src_vec, tgt_vec);
  for (int i = 1; i < stress; i++) {
    matcher.clear();
    matcher.reset();
    matcher.resetSolver();
    solution = matcher.estimate(src_vec, tgt_vec);
    std::cout << "stress iteration:" << i << std::endl;
  }

  // Visualization
  pcl::PointCloud<pcl::PointXYZ> src_viz = *src_pcl;
  pcl::PointCloud<pcl::PointXYZ> tgt_viz = *tgt_pcl;
  pcl::PointCloud<pcl::PointXYZ> est_viz;

  Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
  solution_eigen.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

  matcher.print();

  size_t num_rot_inliers   = matcher.getNumRotationInliers();
  size_t num_final_inliers = matcher.getNumFinalInliers();

  // NOTE(hlim): By checking the final inliers, we can determine whether
  // the registration was successful or not. The larger the threshold,
  // the more conservatively the decision is made.
  // See https://github.com/MIT-SPARK/KISS-Matcher/issues/24
  size_t thres_num_inliers = 5;
  if (num_final_inliers < thres_num_inliers) {
    std::cout << "\033[1;33m=> Registration might have failed :(\033[0m\n";
  } else {
    std::cout << "\033[1;32m=> Registration likely succeeded XD\033[0m\n";
  }

  std::cout << solution_eigen << std::endl;
  std::cout << "=====================================" << std::endl;

  // ------------------------------------------------------------
  // Save warped source cloud
  // ------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr est_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*src_pcl, *est_cloud, solution_eigen);
  std::filesystem::path src_file_path(src_path);
  std::string warped_pcd_filename =
      src_file_path.parent_path().string() + "/" + src_file_path.stem().string() + "_warped.pcd";
  pcl::io::savePCDFileASCII(warped_pcd_filename, *est_cloud);
  std::cout << "Saved transformed source point cloud to: " << warped_pcd_filename << std::endl;

  // ------------------------------------------------------------
  // Visualization
  // ------------------------------------------------------------
  pcl::transformPointCloud(src_viz, est_viz, solution_eigen);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_q_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(src_viz, *src_colored, {195, 195, 195});
  colorize(tgt_viz, *tgt_colored, {89, 167, 230});
  colorize(est_viz, *est_q_colored, {238, 160, 61});

  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
  viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_q_colored, "est_q_blue");

  viewer1.spin();
}
