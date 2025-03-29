#pragma once

#include <chrono>
#include <stdexcept>
#include <string>

namespace kiss_matcher {
class TicToc {
 public:
  TicToc() { tic(); }

  // Reset the timer
  void tic() { start_ = std::chrono::steady_clock::now(); }

  // Return elapsed time in milliseconds or seconds
  double toc(const std::string& unit = "msec") const {
    const auto end                                = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start_;

    if (unit == "msec") {
      return elapsed_seconds.count() * 1000.0;
    } else if (unit == "sec") {
      return elapsed_seconds.count();
    } else {
      throw std::invalid_argument("Unsupported unit: " + unit + ". Use 'msec' or 'sec'.");
    }
  }

 private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
};
}  // namespace kiss_matcher
