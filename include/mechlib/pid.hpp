#pragma once

namespace mechlib {

// The three PID numbers. Output is in millivolts (12000 = full power).
struct PIDGains {
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
};

// A PID controller. Make a new one for each move, or call reset() before reusing it.
class PID {
 public:
  // integralZone: I only builds up when |error| is smaller than this.
  // maxIntegralOutput: the I term never adds more than this many millivolts.
  PID(PIDGains gains, double integralZone, double maxIntegralOutput);

  // error = target - current. dt = seconds since the last call.
  double update(double error, double dt);
  void reset();

 private:
  PIDGains gains_;
  double integralZone_;
  double maxIntegralOutput_;
  double integral_ = 0.0;
  double previousError_ = 0.0;
  bool firstLoop_ = true;
};

}  // namespace mechlib
