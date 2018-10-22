//
// Created by daeyun on 6/14/17.
//
#pragma once

#include <ratio>
#include <iomanip>

namespace scene3d {

constexpr double kMicro = std::micro::num / static_cast<double>(std::micro::den);

long MicroSecondsSinceEpoch();

template<typename Unit=std::ratio<1, 1>>
double TimeSinceEpoch() {
  constexpr double ratio = kMicro * Unit::den / Unit::num;
  return MicroSecondsSinceEpoch() * ratio;
}

class RunningAverageTimer {
 public:
  explicit RunningAverageTimer(const std::string &name);

  void Tic();

  void Toc();

  template<typename Unit=std::ratio<1, 1>>
  double OperationsPer() {
    constexpr double ratio = Unit::num / (kMicro * Unit::den);
    return running_average_microseconds_ * ratio;
  }

  template<typename Unit=std::ratio<1, 1>>
  double Duration() {
    constexpr double ratio = kMicro * Unit::den / Unit::num;
    return running_average_microseconds_ * ratio;
  }

  static constexpr double kEmaDecay = 0.98;

 private:
  double running_average_microseconds_ = -1;
  long last_seen_;
  const std::string name_;
};

class SimpleTimer {
 public:
  explicit SimpleTimer(const std::string &name);

  void Tic();

  void Toc();

  template<typename Unit=std::ratio<1, 1>>
  double OperationsPer() {
    constexpr double ratio = Unit::num / (kMicro * Unit::den);
    return static_cast<double>(sum_microseconds_) / toc_count_ * ratio;
  }

  template<typename Unit=std::ratio<1, 1>>
  double Duration() {
    return TotalElapsed<Unit>() / toc_count_;
  }

  template<typename Unit=std::ratio<1, 1>>
  double TotalElapsed() {
    constexpr double ratio = kMicro * Unit::den / Unit::num;
    return static_cast<double>(sum_microseconds_) * ratio;
  }

  const std::string &name() const { return name_; }

 private:
  unsigned long sum_microseconds_ = 0;
  unsigned long toc_count_ = 0;
  long last_seen_;
  const std::string name_;
};

}
