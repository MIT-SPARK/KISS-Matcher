#pragma once

#ifndef KISS_MATCHER_LOOP_TYPES_H
#define KISS_MATCHER_LOOP_TYPES_H

#include <cstddef>  // for size_t
#include <limits>   // for std::numeric_limits
#include <utility>
#include <vector>

struct LoopCandidate {
  bool found_      = false;
  size_t idx_      = std::numeric_limits<size_t>::max();
  double distance_ = std::numeric_limits<double>::max();
};

using LoopCandidates = std::vector<LoopCandidate>;

using LoopIdxPair = std::pair<size_t, size_t>;

using LoopIdxPairs = std::vector<LoopIdxPair>;

#endif  // KISS_MATCHER_LOOP_TYPES_H
