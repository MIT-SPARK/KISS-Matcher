#include "slam/pose_graph_manager.h"

using namespace kiss_matcher;

PoseGraphManager::PoseGraphManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("km_sam", options) {
  double loop_pub_hz;
  double loop_detector_hz;
  double loop_nnsearch_hz;
  double map_update_hz;
  double vis_hz;

  LoopClosureConfig lc_config;
  LoopDetectorConfig ld_config;
  auto &gc = lc_config.gicp_config_;
  auto &mc = lc_config.matcher_config_;

  map_frame_             = declare_parameter<std::string>("map_frame", "map");
  base_frame_            = declare_parameter<std::string>("base_frame", "base");
  loop_pub_hz            = declare_parameter<double>("loop_pub_hz", 0.1);
  loop_detector_hz       = declare_parameter<double>("loop_detector_hz", 1.0);
  loop_nnsearch_hz       = declare_parameter<double>("loop_nnsearch_hz", 1.0);
  loop_pub_delayed_time_ = declare_parameter<double>("loop_pub_delayed_time", 60.0);
  map_update_hz          = declare_parameter<double>("map_update_hz", 0.2);
  vis_hz                 = declare_parameter<double>("vis_hz", 0.5);

  lc_config.voxel_res_             = declare_parameter<double>("voxel_resolution", 0.3);
  scan_voxel_res_                  = lc_config.voxel_res_;
  map_voxel_res_                   = declare_parameter<double>("map_voxel_resolution", 1.0);
  save_voxel_res_                  = declare_parameter<double>("save_voxel_resolution", 0.3);
  keyframe_thr_                    = declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
  lc_config.num_submap_keyframes_  = declare_parameter<int>("keyframe.num_submap_keyframes", 5);
  lc_config.verbose_               = declare_parameter<bool>("loop.verbose", false);
  lc_config.is_multilayer_env_     = declare_parameter<bool>("loop.is_multilayer_env", false);
  lc_config.loop_detection_radius_ = declare_parameter<double>("loop.loop_detection_radius", 15.0);
  lc_config.loop_detection_timediff_threshold_ =
      declare_parameter<double>("loop.loop_detection_timediff_threshold", 10.0);

  gc.num_threads_               = declare_parameter<int>("local_reg.num_threads", 8);
  gc.correspondence_randomness_ = declare_parameter<int>("local_reg.correspondences_number", 20);
  gc.max_num_iter_              = declare_parameter<int>("local_reg.max_num_iter", 32);
  gc.scale_factor_for_corr_dist_ =
      declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
  gc.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

  lc_config.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
  lc_config.num_inliers_threshold_ =
      declare_parameter<int>("global_reg.num_inliers_threshold", 100);

  save_map_bag_         = declare_parameter<bool>("result.save_map_bag", false);
  save_map_pcd_         = declare_parameter<bool>("result.save_map_pcd", false);
  save_in_kitti_format_ = declare_parameter<bool>("result.save_in_kitti_format", false);
  seq_name_             = declare_parameter<std::string>("result.seq_name", "");

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  package_path_ = "";

  loop_closure_          = std::make_shared<LoopClosure>(lc_config, this->get_logger());
  loop_detection_radius_ = lc_config.loop_detection_radius_;

  loop_detector_ = std::make_shared<LoopDetector>(ld_config, this->get_logger());

  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip      = 1;
  isam_handler_                     = std::make_shared<gtsam::ISAM2>(isam_params_);

  odom_path_.header.frame_id      = map_frame_;
  corrected_path_.header.frame_id = map_frame_;

  // NOTE(hlim): To make this node compatible with being launched under different namespaces,
  // I deliberately avoided adding a '/' in front of the topic names.
  path_pub_           = this->create_publisher<nav_msgs::msg::Path>("path/original", 10);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path/corrected", 10);
  map_pub_            = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);
  scan_pub_           = this->create_publisher<sensor_msgs::msg::PointCloud2>("curr_scan", 10);
  loop_detection_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("loop_detection", 10);
  loop_detection_radius_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("loop_detection_radius", 10);

  // loop_closures_pub_ =
  // this->create_publisher<pose_graph_tools_msgs::msg::PoseGraph>("/hydra_ros_node/external_loop_closures",
  // 10);
  realtime_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", 10);
  debug_src_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/src", 10);
  debug_tgt_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/tgt", 10);
  debug_coarse_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/coarse_alignment", 10);
  debug_fine_aligned_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/fine_alignment", 10);
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/debug_cloud", 10);

  sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odom");
  sub_scan_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/cloud");

  sub_node_ = std::make_shared<message_filters::Synchronizer<NodeSyncPolicy>>(
      NodeSyncPolicy(10), *sub_odom_, *sub_scan_);
  sub_node_->registerCallback(std::bind(
      &PoseGraphManager::callbackNode, this, std::placeholders::_1, std::placeholders::_2));

  sub_save_flag_ = this->create_subscription<std_msgs::msg::String>(
      "save_dir", 1, std::bind(&PoseGraphManager::saveFlagCallback, this, std::placeholders::_1));

  // hydra_loop_timer_ = this->create_wall_timer(
  //   std::chrono::duration<double>(1.0 / loop_pub_hz),
  //   std::bind(&PoseGraphManager::loopPubTimerFunc, this));

  map_cloud_.reset(new pcl::PointCloud<PointType>());
  map_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / map_update_hz),
                                       std::bind(&PoseGraphManager::buildMap, this));

  loop_detector_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_detector_hz),
                              std::bind(&PoseGraphManager::detectLoopClosureByLoopDetector, this));

  loop_nnsearch_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_nnsearch_hz),
                              std::bind(&PoseGraphManager::detectLoopClosureByNNSearch, this));

  graph_vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                              std::bind(&PoseGraphManager::visualizePoseGraph, this));

  lc_reg_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0),
                                          std::bind(&PoseGraphManager::performRegistration, this));

  // 20 Hz is enough as long as it's faster than the full registration process.
  lc_vis_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                              std::bind(&PoseGraphManager::visualizeLoopClosureClouds, this));

  if (!lc_config.is_multilayer_env_) {
    RCLCPP_WARN(
        get_logger(),
        "'loop.is_multilayer_env' is set to `false`. "
        "This setting is recommended for outdoor environments to ignore the effect of Z-drift. "
        "However, if you're running SLAM in an indoor multi-layer environment, "
        "consider setting it to true to enable full 3D NN search for loop candidates.");
  }
  RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

PoseGraphManager::~PoseGraphManager() {
  if (save_map_bag_) {
    RCLCPP_INFO(this->get_logger(), "NOTE(hlim): skipping final bag save in ROS2 example code.");
  }
  if (save_map_pcd_) {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].scan_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map += transformPcd(keyframes_[i].scan_, keyframes_[i].pose_corrected_);
      }
    }
    const auto &voxelized_map = voxelize(corrected_map, save_voxel_res_);
    pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
    RCLCPP_INFO(this->get_logger(), "Result saved in .pcd format (Destructor).");
  }
}

void PoseGraphManager::appendKeyframePose(const PoseGraphNode &node) {
  odoms_.points.emplace_back(node.pose_(0, 3), node.pose_(1, 3), node.pose_(2, 3));

  corrected_odoms_.points.emplace_back(
      node.pose_corrected_(0, 3), node.pose_corrected_(1, 3), node.pose_corrected_(2, 3));

  odom_path_.poses.emplace_back(eigenToPoseStamped(node.pose_, map_frame_));
  corrected_path_.poses.emplace_back(eigenToPoseStamped(node.pose_corrected_, map_frame_));
  return;
}

void PoseGraphManager::callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg) {
  static size_t latest_keyframe_idx = 0;

  Eigen::Matrix4d lastest_odom = current_frame_.pose_;
  current_frame_               = PoseGraphNode(*odom_msg, *scan_msg, latest_keyframe_idx);

  kiss_matcher::TicToc total_timer;
  kiss_matcher::TicToc local_timer;

  visualizeCurrentData(lastest_odom, odom_msg->header.stamp, scan_msg->header.frame_id);

  if (!is_initialized_) {
    keyframes_.push_back(current_frame_);
    appendKeyframePose(current_frame_);

    auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
        gtsam::noiseModel::Diagonal::Variances(variance_vector);

    gtsam_graph_.add(
        gtsam::PriorFactor<gtsam::Pose3>(0, eigenToGtsam(current_frame_.pose_), prior_noise));

    init_esti_.insert(latest_keyframe_idx, eigenToGtsam(current_frame_.pose_));
    ++latest_keyframe_idx;
    is_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "The first node comes. Initialization complete.");

  } else {
    const auto t_keyframe_processing = local_timer.toc();
    if (checkIfKeyframe(current_frame_, keyframes_.back())) {
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_.push_back(current_frame_);
      }

      auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise =
          gtsam::noiseModel::Diagonal::Variances(variance_vector);

      gtsam::Pose3 pose_from = eigenToGtsam(keyframes_[latest_keyframe_idx - 1].pose_corrected_);
      gtsam::Pose3 pose_to   = eigenToGtsam(current_frame_.pose_corrected_);

      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            latest_keyframe_idx - 1, latest_keyframe_idx, pose_from.between(pose_to), odom_noise));
        init_esti_.insert(latest_keyframe_idx, pose_to);
      }

      ++latest_keyframe_idx;
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        appendKeyframePose(current_frame_);
      }

      local_timer.tic();
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        isam_handler_->update(gtsam_graph_, init_esti_);
        isam_handler_->update();
        if (loop_closure_added_) {
          isam_handler_->update();
          isam_handler_->update();
          isam_handler_->update();
        }
        gtsam_graph_.resize(0);
        init_esti_.clear();
      }
      const auto t_optim = local_timer.toc();

      {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        corrected_esti_ = isam_handler_->calculateEstimate();
        last_corrected_pose_ =
            gtsamToEigen(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
        odom_delta_ = Eigen::Matrix4d::Identity();
      }
      if (loop_closure_added_) {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < corrected_esti_.size(); ++i) {
          keyframes_[i].pose_corrected_ = gtsamToEigen(corrected_esti_.at<gtsam::Pose3>(i));
        }
        loop_closure_added_ = false;
      }

      const auto t_total = total_timer.toc();

      RCLCPP_INFO(
          this->get_logger(),
          "# of Keyframes: %zu. Timing (msec) → Total: %.1f | Keyframe: %.1f | Optim.: %.1f",
          keyframes_.size(),
          t_total,
          t_keyframe_processing,
          t_optim);
    }
  }
}

// void PoseGraphManager::loopPubTimerFunc()
// {
//   if (loop_msgs_.edges.empty()) {
//     RCLCPP_WARN(this->get_logger(),
//       "`loop_msgs_.edges` is empty. Skipping loop closure publishing.");
//     return;
//   }
//   if (last_lc_time_ + loop_pub_delayed_time_ < this->now().seconds()) {
//     loop_closures_pub_->publish(loop_msgs_);
//     loop_msgs_.nodes.clear();
//     loop_msgs_.edges.clear();
//     RCLCPP_INFO(this->get_logger(), "`loop_msgs_` is successfully published!");
//     return;
//   }
// }

void PoseGraphManager::buildMap() {
  static size_t start_idx = 0;

  if (map_pub_->get_subscription_count() > 0) {
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      if (need_map_update_) {
        map_cloud_->clear();
        start_idx = 0;
      }

      if (keyframes_.empty()) return;

      // NOTE(hlim): Building the full map causes RViz delay when keyframes > 500.
      // Since the map is for visualization only, we apply a heuristic to reduce cost.
      for (size_t i = start_idx; i < keyframes_.size(); ++i) {
        if (keyframes_[i].voxelized_scan_.empty()) {
          keyframes_[i].voxelized_scan_ = *voxelize(keyframes_[i].scan_, scan_voxel_res_);
        }
        *map_cloud_ += transformPcd(keyframes_[i].voxelized_scan_, keyframes_[i].pose_corrected_);
      }

      start_idx = keyframes_.size();
    }

    const auto &voxelized_map = voxelize(map_cloud_, map_voxel_res_);
    map_pub_->publish(toROSMsg(*voxelized_map, map_frame_));
  }

  if (need_map_update_) {
    need_map_update_ = false;
  }
}

void PoseGraphManager::detectLoopClosureByLoopDetector() {
  auto &query = keyframes_.back();
  if (!is_initialized_ || keyframes_.empty() || query.loop_detector_processed_) {
    return;
  }
  query.loop_detector_processed_ = true;

  kiss_matcher::TicToc ld_timer;
  const auto &loop_idx_pairs = loop_detector_->fetchLoopCandidates(query, keyframes_);

  for (const auto &loop_candidate : loop_idx_pairs) {
    loop_idx_pair_queue_.push(loop_candidate);
  }

  const auto t_ld = ld_timer.toc();
}

void PoseGraphManager::detectLoopClosureByNNSearch() {
  auto &query = keyframes_.back();
  if (!is_initialized_ || keyframes_.empty() || query.nnsearch_processed_) {
    return;
  }
  query.nnsearch_processed_ = true;

  const auto &loop_idx_pairs = loop_closure_->fetchLoopCandidates(query, keyframes_);

  for (const auto &loop_candidate : loop_idx_pairs) {
    loop_idx_pair_queue_.push(loop_candidate);
  }
}

void PoseGraphManager::performRegistration() {
  kiss_matcher::TicToc reg_timer;
  if (loop_idx_pair_queue_.empty()) {
    return;
  }
  const auto [query_idx, match_idx] = loop_idx_pair_queue_.front();
  loop_idx_pair_queue_.pop();

  const RegOutput &reg_output = loop_closure_->performLoopClosure(keyframes_, query_idx, match_idx);
  need_lc_cloud_vis_update_   = true;

  if (reg_output.is_valid_) {
    RCLCPP_INFO(this->get_logger(), "LC accepted. Overlapness: %.3f", reg_output.overlapness_);
    gtsam::Pose3 pose_from = eigenToGtsam(reg_output.pose_ * keyframes_[query_idx].pose_corrected_);
    gtsam::Pose3 pose_to   = eigenToGtsam(keyframes_[match_idx].pose_corrected_);

    // TODO(hlim): Parameterize
    auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    gtsam::noiseModel::Diagonal::shared_ptr loop_noise =
        gtsam::noiseModel::Diagonal::Variances(variance_vector);

    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          query_idx, match_idx, pose_from.between(pose_to), loop_noise));
    }

    vis_loop_edges_.emplace_back(query_idx, match_idx);
    loop_closure_added_    = true;
    need_map_update_       = true;
    need_graph_vis_update_ = true;

    // --------------------------------------------------
    // TODO(hlim): resurrect pose_graph_tools_msgs
    // pose_graph_tools_msgs::msg::PoseGraphEdge edge;
    // double lidar_end_time_compensation = 0.1;
    // edge.header.stamp = this->now();
    // edge.robot_from = 0;
    // edge.robot_to = 0;
    // edge.type = 1;

    // edge.key_to = static_cast<uint64_t>(
    //   (keyframes_.back().timestamp_ - lidar_end_time_compensation) * 1e9);
    // edge.key_from = static_cast<uint64_t>(
    //   (keyframes_[closest_keyframe_idx].timestamp_ - lidar_end_time_compensation) * 1e9);

    // Eigen::Matrix4d pose_inv = pose_to.matrix().inverse() * pose_from.matrix();
    // edge.pose = poseEigToPoseGeo(pose_inv);
    // loop_msgs_.edges.emplace_back(edge);
    // last_lc_time_ = this->now().seconds();
    // --------------------------------------------------
  } else {
    if (reg_output.overlapness_ == 0.0) {
      RCLCPP_WARN(this->get_logger(), "LC rejected. KISS-Matcher failed");
    } else {
      RCLCPP_WARN(this->get_logger(), "LC rejected. Overlapness: %.3f", reg_output.overlapness_);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Reg: %.1f msec", reg_timer.toc());
}

void PoseGraphManager::visualizeCurrentData(const Eigen::Matrix4d &lastest_odom,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id) {
  // NOTE(hlim): Instead of visualizing only when adding keyframes (node-wise), which can feel
  // choppy, we visualize the current frame every cycle to ensure smoother, real-time visualization.
  {
    std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
    odom_delta_                    = odom_delta_ * lastest_odom.inverse() * current_frame_.pose_;
    current_frame_.pose_corrected_ = last_corrected_pose_ * odom_delta_;

    geometry_msgs::msg::PoseStamped ps =
        eigenToPoseStamped(current_frame_.pose_corrected_, map_frame_);
    realtime_pose_pub_->publish(ps);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp    = timestamp;
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id  = base_frame_.empty() ? frame_id : base_frame_;
    Eigen::Quaterniond q(current_frame_.pose_corrected_.block<3, 3>(0, 0));
    transform_stamped.transform.translation.x = current_frame_.pose_corrected_(0, 3);
    transform_stamped.transform.translation.y = current_frame_.pose_corrected_(1, 3);
    transform_stamped.transform.translation.z = current_frame_.pose_corrected_(2, 3);
    transform_stamped.transform.rotation.x    = q.x();
    transform_stamped.transform.rotation.y    = q.y();
    transform_stamped.transform.rotation.z    = q.z();
    transform_stamped.transform.rotation.w    = q.w();
    tf_broadcaster_->sendTransform(transform_stamped);
  }

  scan_pub_->publish(
      toROSMsg(transformPcd(current_frame_.scan_, current_frame_.pose_corrected_), map_frame_));
  if (!corrected_path_.poses.empty()) {
    loop_detection_radius_pub_->publish(
        visualizeLoopDetectionRadius(corrected_path_.poses.back().pose.position));
  }
}

void PoseGraphManager::visualizePoseGraph() {
  if (!is_initialized_) {
    return;
  }

  if (need_graph_vis_update_) {
    gtsam::Values corrected_esti_copied;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
    nav_msgs::msg::Path corrected_path;

    {
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      corrected_esti_copied = corrected_esti_;
    }
    for (size_t i = 0; i < corrected_esti_copied.size(); ++i) {
      gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
      corrected_odoms.points.emplace_back(
          pose_.translation().x(), pose_.translation().y(), pose_.translation().z());

      corrected_path.poses.push_back(gtsamToPoseStamped(pose_, map_frame_));
    }
    if (!vis_loop_edges_.empty()) {
      loop_detection_pub_->publish(visualizeLoopMarkers(corrected_esti_copied));
    }
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odoms_      = corrected_odoms;
      corrected_path_.poses = corrected_path.poses;
    }
    need_graph_vis_update_ = false;
  }

  {
    std::lock_guard<std::mutex> lock(vis_mutex_);
    path_pub_->publish(odom_path_);
    corrected_path_pub_->publish(corrected_path_);
  }
}

void PoseGraphManager::visualizeLoopClosureClouds() {
  if (!need_lc_cloud_vis_update_) {
    return;
  }

  debug_src_pub_->publish(toROSMsg(loop_closure_->getSourceCloud(), map_frame_));
  debug_tgt_pub_->publish(toROSMsg(loop_closure_->getTargetCloud(), map_frame_));
  debug_fine_aligned_pub_->publish(toROSMsg(loop_closure_->getFinalAlignedCloud(), map_frame_));
  debug_coarse_aligned_pub_->publish(toROSMsg(loop_closure_->getCoarseAlignedCloud(), map_frame_));
  debug_cloud_pub_->publish(toROSMsg(loop_closure_->getDebugCloud(), map_frame_));
  need_lc_cloud_vis_update_ = false;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopMarkers(
    const gtsam::Values &corrected_poses) const {
  visualization_msgs::msg::Marker edges;
  edges.type               = visualization_msgs::msg::Marker::LINE_LIST;
  edges.scale.x            = 0.12f;
  edges.header.frame_id    = map_frame_;
  edges.pose.orientation.w = 1.0f;
  edges.color.r            = 1.0f;
  edges.color.g            = 1.0f;
  edges.color.b            = 1.0f;
  edges.color.a            = 1.0f;

  for (size_t i = 0; i < vis_loop_edges_.size(); ++i) {
    if (vis_loop_edges_[i].first >= corrected_poses.size() ||
        vis_loop_edges_[i].second >= corrected_poses.size()) {
      continue;
    }
    gtsam::Pose3 pose  = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].first);
    gtsam::Pose3 pose2 = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].second);

    geometry_msgs::msg::Point p, p2;
    p.x  = pose.translation().x();
    p.y  = pose.translation().y();
    p.z  = pose.translation().z();
    p2.x = pose2.translation().x();
    p2.y = pose2.translation().y();
    p2.z = pose2.translation().z();

    edges.points.push_back(p);
    edges.points.push_back(p2);
  }
  return edges;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopDetectionRadius(
    const geometry_msgs::msg::Point &latest_position) const {
  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = map_frame_;
  sphere.id              = 100000;  // arbitrary number
  sphere.type            = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose.position.x = latest_position.x;
  sphere.pose.position.y = latest_position.y;
  sphere.pose.position.z = latest_position.z;
  sphere.scale.x         = 2 * loop_detection_radius_;
  sphere.scale.y         = 2 * loop_detection_radius_;
  sphere.scale.z         = 2 * loop_detection_radius_;
  // Use transparanet cyan color
  sphere.color.r = 0.0;
  sphere.color.g = 0.824;
  sphere.color.b = 1.0;
  sphere.color.a = 0.5;

  return sphere;
}

bool PoseGraphManager::checkIfKeyframe(const PoseGraphNode &query_node,
                                       const PoseGraphNode &latest_node) {
  return keyframe_thr_ < (latest_node.pose_corrected_.block<3, 1>(0, 3) -
                          query_node.pose_corrected_.block<3, 1>(0, 3))
                             .norm();
}

void PoseGraphManager::saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg) {
  std::string save_dir        = !msg->data.empty() ? msg->data : package_path_;
  std::string seq_directory   = save_dir + "/" + seq_name_;
  std::string scans_directory = seq_directory + "/scans";

  if (save_in_kitti_format_) {
    RCLCPP_INFO(this->get_logger(),
                "Scans are saved in %s, following the KITTI and TUM format",
                scans_directory.c_str());

    if (fs::exists(seq_directory)) {
      fs::remove_all(seq_directory);
    }
    fs::create_directories(scans_directory);

    std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
    std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
    tum_pose_file << "#timestamp x y z qx qy qz qw\n";

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        std::stringstream ss_;
        ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
        RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
        pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].scan_);

        const auto &pose_ = keyframes_[i].pose_corrected_;
        kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                        << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                        << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                        << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

        const auto &lidar_optim_pose_ =
            eigenToPoseStamped(keyframes_[i].pose_corrected_, map_frame_);
        tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_ << " "
                      << lidar_optim_pose_.pose.position.x << " "
                      << lidar_optim_pose_.pose.position.y << " "
                      << lidar_optim_pose_.pose.position.z << " "
                      << lidar_optim_pose_.pose.orientation.x << " "
                      << lidar_optim_pose_.pose.orientation.y << " "
                      << lidar_optim_pose_.pose.orientation.z << " "
                      << lidar_optim_pose_.pose.orientation.w << "\n";
      }
    }
    kitti_pose_file.close();
    tum_pose_file.close();
    RCLCPP_INFO(this->get_logger(), "Scans and poses saved in .pcd and KITTI format");
  }
  if (save_map_bag_) {
    RCLCPP_INFO(this->get_logger(),
                "NOTE(hlim): rosbag2 saving not directly implemented; skipping.");
  }
  if (save_map_pcd_) {
    pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
    corrected_map->reserve(keyframes_[0].scan_.size() * keyframes_.size());

    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        *corrected_map += transformPcd(keyframes_[i].scan_, keyframes_[i].pose_corrected_);
      }
    }
    const auto &voxelized_map = voxelize(corrected_map, save_voxel_res_);
    pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd",
                                         *voxelized_map);
    RCLCPP_INFO(this->get_logger(), "Accumulated map cloud saved in .pcd format");
  }
}

// ----------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<PoseGraphManager>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
