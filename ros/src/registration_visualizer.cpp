// An example showing KISS-Matcher registration with animated transformation visualization.
#include <Eigen/Geometry>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudVisualizer : public rclcpp::Node {
 public:
  PointCloudVisualizer() : Node("registration_visualizer") {
    declareAndGetParameters();

    source_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/src_cloud", 10);
    target_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tgt_cloud", 10);

    loadPointClouds();
    computeRegistration();
    animateTransformation();
  }

 private:
  std::string base_dir_;
  std::string src_pcd_path_;
  std::string tgt_pcd_path_;
  double resolution_;
  double moving_rate_;
  double frame_rate_;
  double scale_factor_;
  Eigen::Matrix4f estimated_transform_ = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_, target_publisher_;

  void declareAndGetParameters() {
    declare_parameter("base_dir", "");
    declare_parameter("src_pcd_path", "");
    declare_parameter("tgt_pcd_path", "");
    declare_parameter("resolution", 0.1);
    declare_parameter("moving_rate", 10.0);
    declare_parameter("frame_rate", 10.0);
    declare_parameter("scale_factor", 1.0);

    get_parameter("base_dir", base_dir_);
    get_parameter("src_pcd_path", src_pcd_path_);
    get_parameter("tgt_pcd_path", tgt_pcd_path_);
    get_parameter("resolution", resolution_);
    get_parameter("moving_rate", moving_rate_);
    get_parameter("frame_rate", frame_rate_);
    get_parameter("scale_factor", scale_factor_);
  }

  void loadPointClouds() {
    auto logger = get_logger();

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(base_dir_ + src_pcd_path_, *source_cloud_) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(base_dir_ + tgt_pcd_path_, *target_cloud_) == -1) {
      RCLCPP_ERROR(logger, "Failed to load PCD files.");
      return;
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*source_cloud_, *source_cloud_, indices);
    pcl::removeNaNFromPointCloud(*target_cloud_, *target_cloud_, indices);

    Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
    scale_transform.block<3, 3>(0, 0) *= scale_factor_;
    pcl::transformPointCloud(*source_cloud_, *source_cloud_, scale_transform);
    pcl::transformPointCloud(*target_cloud_, *target_cloud_, scale_transform);

    RCLCPP_INFO(logger, "PCD files loaded and downsampled successfully.");
  }

  std::vector<Eigen::Vector3f> convertCloudToEigenVector(
      const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      points.emplace_back(point.x, point.y, point.z);
    }
    return points;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr convertEigenVectorToCloud(
      const std::vector<Eigen::Vector3f>& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(points.size());
    for (const auto& point : points) {
      cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    return cloud;
  }

  void computeRegistration() {
    auto logger = get_logger();

    auto source_points = convertCloudToEigenVector(*source_cloud_);
    auto target_points = convertCloudToEigenVector(*target_cloud_);

    if (source_points.empty() || target_points.empty()) {
      RCLCPP_ERROR(logger, "Error: One of the point clouds is empty after filtering.");
      return;
    }

    RCLCPP_INFO(logger, "Running KISSMatcher...");

    kiss_matcher::KISSMatcherConfig config(resolution_);
    kiss_matcher::KISSMatcher matcher(config);
    auto solution = matcher.estimate(source_points, target_points);

    estimated_transform_.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
    estimated_transform_.topRightCorner(3, 1) = solution.translation.cast<float>();

    matcher.print();

    constexpr size_t min_inliers = 5;
    if (matcher.getNumFinalInliers() < min_inliers) {
      RCLCPP_WARN(logger, "=> Registration might have failed.");
    } else {
      RCLCPP_INFO(logger, "=> Registration likely succeeded.");
    }

    source_cloud_ = convertEigenVectorToCloud(source_points);
    target_cloud_ = convertEigenVectorToCloud(target_points);
  }

  void animateTransformation() {
    rclcpp::Rate loop_rate(frame_rate_);
    sensor_msgs::msg::PointCloud2 target_msg, transformed_source_msg;

    Eigen::Vector4d source_centroid;
    pcl::compute3DCentroid(*source_cloud_, source_centroid);

    Eigen::Matrix4f translation_to_origin   = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translation_back        = Eigen::Matrix4f::Identity();
    translation_to_origin.block<3, 1>(0, 3) = -source_centroid.head<3>().cast<float>();
    translation_back.block<3, 1>(0, 3)      = source_centroid.head<3>().cast<float>();

    Eigen::Matrix4f accumulated_transform = Eigen::Matrix4f::Identity();

    int num_total_steps = 2 * moving_rate_;

    for (double step = 0; step <= num_total_steps; ++step) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr animated_cloud =
          std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      Eigen::Matrix4f frame_transform = Eigen::Matrix4f::Identity();

      if (step <= num_total_steps / 2) {
        double progress = static_cast<double>(step) / (num_total_steps / 2);
        Eigen::Quaternionf start_rotation(Eigen::Matrix3f::Identity());
        Eigen::Quaternionf goal_rotation(estimated_transform_.block<3, 3>(0, 0));
        Eigen::Quaternionf interpolated_rotation = start_rotation.slerp(progress, goal_rotation);

        Eigen::Matrix4f rotation_transform   = Eigen::Matrix4f::Identity();
        rotation_transform.block<3, 3>(0, 0) = interpolated_rotation.toRotationMatrix();

        frame_transform       = translation_back * rotation_transform * translation_to_origin;
        accumulated_transform = frame_transform;
      } else {
        double translation_ratio =
            static_cast<double>(step - num_total_steps / 2) / (num_total_steps / 2);
        Eigen::Vector3f start_translation = accumulated_transform.block<3, 1>(0, 3);
        Eigen::Vector3f goal_translation  = estimated_transform_.block<3, 1>(0, 3);

        Eigen::Vector3f interpolated_translation =
            start_translation + translation_ratio * (goal_translation - start_translation);

        frame_transform                   = accumulated_transform;
        frame_transform.block<3, 1>(0, 3) = interpolated_translation;
      }

      pcl::transformPointCloud(*source_cloud_, *animated_cloud, frame_transform);
      pcl::toROSMsg(*animated_cloud, transformed_source_msg);
      pcl::toROSMsg(*target_cloud_, target_msg);
      transformed_source_msg.header.frame_id = "map";
      target_msg.header.frame_id             = "map";
      target_publisher_->publish(target_msg);
      source_publisher_->publish(transformed_source_msg);

      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudVisualizer>());
  rclcpp::shutdown();
  return 0;
}
