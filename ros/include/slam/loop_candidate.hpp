#pragma once

#ifndef KISS_MATCHER_LOOP_CANDIDATE_H
#define KISS_MATCHER_LOOP_CANDIDATE_H

#include <cstddef>  // for size_t
#include <limits>   // for std::numeric_limits

struct LoopCandidate {
  bool found_      = false;
  size_t idx_      = 0;
  double distance_ = std::numeric_limits<double>::max();
};

#endif  // KISS_MATCHER_LOOP_CANDIDATE_H
