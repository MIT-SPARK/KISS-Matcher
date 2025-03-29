#include "slam/loop_detector.h"

using namespace kiss_matcher;

LoopDetector::LoopDetector(const LoopDetectorConfig &config, const rclcpp::Logger &logger)
    : config_(config), logger_(logger) {
  // Fill your declaration here
}

LoopDetector::~LoopDetector() {}

LoopCandidate LoopDetector::fetchLoopCandidate(const PoseGraphNode &query_frame,
                                               const std::vector<PoseGraphNode> &keyframes) {
  LoopCandidate candidate;
  //------------------------------------------------------------
  // Implement your loop detector here
  //------------------------------------------------------------
  return candidate;
}
