#include "slam/loop_detector.h"

using namespace kiss_matcher;

LoopDetector::LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger)
    : config_(config), logger_(logger) {
  // Fill your declaration here
}

LoopDetector::~LoopDetector() {}

size_t LoopDetector::fetchLoopCandidateIdx(const PoseGraphNode &front_keyframe,
                                           const std::vector<PoseGraphNode> &keyframes) {
  size_t closest_idx = 0;

  return closest_idx;
}
