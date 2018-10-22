//
// Created by daeyun on 6/14/17.
//

#include "benchmark.h"

#include <chrono>

#include <glog/logging.h>
#include <gsl/gsl_assert>

namespace scene3d {
long MicroSecondsSinceEpoch() {
  return std::chrono::duration_cast<std::chrono::microseconds>
      (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

void RunningAverageTimer::Toc() {
  auto now = MicroSecondsSinceEpoch();
  auto elapsed = now - last_seen_;
  last_seen_ = now;

  if (GSL_UNLIKELY(running_average_microseconds_ < 0)) {
    running_average_microseconds_ = elapsed;
  } else {
    running_average_microseconds_ = running_average_microseconds_ * kEmaDecay + elapsed * (1 - kEmaDecay);
  }
}

void RunningAverageTimer::Tic() {
  last_seen_ = MicroSecondsSinceEpoch();
}

RunningAverageTimer::RunningAverageTimer(const std::string &name) : name_(name) {
  running_average_microseconds_ = -1;
  last_seen_ = MicroSecondsSinceEpoch();
}

void SimpleTimer::Toc() {
  auto now = MicroSecondsSinceEpoch();
  auto elapsed = now - last_seen_;
  last_seen_ = now;
  sum_microseconds_ += elapsed;
  ++toc_count_;
}

void SimpleTimer::Tic() {
  last_seen_ = MicroSecondsSinceEpoch();
}

SimpleTimer::SimpleTimer(const std::string &name) : name_(name) {
  Tic();
}
}  // namespace mvshape

