#include "mechlib/pid.hpp"

#include <algorithm>
#include <cmath>

namespace mechlib {

PID::PID(PIDGains gains, double integralZone, double maxIntegralOutput)
    : gains_(gains), integralZone_(integralZone), maxIntegralOutput_(maxIntegralOutput) {}

void PID::reset() {
  integral_ = 0.0;
  previousError_ = 0.0;
  firstLoop_ = true;
}

double PID::update(double error, double dt) {
  // P: push in proportion to how far away we are.
  double pTerm = gains_.kP * error;

  // I: add up error over time, but only near the target, and wipe it when we
  // cross the target, so it finishes the move without causing big overshoot.
  if (std::fabs(error) < integralZone_) {
    integral_ += error * dt;
  } else {
    integral_ = 0.0;
  }
  if (!firstLoop_ && (error > 0) != (previousError_ > 0)) integral_ = 0.0;
  if (gains_.kI != 0.0) {
    double maxIntegral = maxIntegralOutput_ / std::fabs(gains_.kI);
    integral_ = std::clamp(integral_, -maxIntegral, maxIntegral);
  }
  double iTerm = gains_.kI * integral_;

  // D: how fast the error is changing. Acts like a brake near the target.
  double derivative = firstLoop_ ? 0.0 : (error - previousError_) / dt;
  double dTerm = gains_.kD * derivative;

  previousError_ = error;
  firstLoop_ = false;
  return pTerm + iTerm + dTerm;
}

}  // namespace mechlib
