#include "mechlib/drive.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "screen.hpp"

namespace mechlib {

namespace {

const int LOOP_MS = 10;
const double LOOP_DT = LOOP_MS / 1000.0;

// When is a move done? Close enough AND nearly stopped, for SETTLE_MS in a row.
const double DRIVE_SETTLE_INCHES = 0.5;
const double DRIVE_SETTLE_SPEED = 2.0;   // inches per second
const double TURN_SETTLE_DEGREES = 1.0;
const double TURN_SETTLE_SPEED = 5.0;    // degrees per second
const int SETTLE_MS = 100;
// Friction can stop a robot just outside those limits. If it stays stopped
// within twice the limit for STOPPED_MS, the move ends instead of waiting for
// the timeout (MoveResult.finalError still shows how far off it was).
const int STOPPED_MS = 250;

// Max increase in drive voltage per loop, so the robot doesn't wheelie or slip.
const double SLEW_MV = 600.0;

// The default gains were tuned in simulation for this reference robot:
// 6 blue motors, 36:48 (450 RPM), 3.25" wheels, 11.5" track width.
// Other robots get these gains scaled to how fast they drive and turn.
const double REF_WHEEL_SPEED = 600.0 * 0.75 / 60.0 * M_PI * 3.25;  // inches per second
const double REF_TRACK_WIDTH = 11.5;
const double REF_TURN_SPEED = 2.0 * REF_WHEEL_SPEED / REF_TRACK_WIDTH * 180.0 / M_PI;  // degrees per second
const PIDGains REF_DRIVE_GAINS = {800.0, 400.0, 60.0};
const PIDGains REF_TURN_GAINS = {120.0, 300.0, 17.0};
const PIDGains REF_HEADING_GAINS = {150.0, 0.0, 8.0};

const char* GAINS_FILE = "/usd/mechlib_gains.txt";
const char* GAINS_HEADER = "mechlib-gains-v1";

// Any angle -> the range -180 to 180, so turns always go the short way.
double wrapAngle(double degrees) {
  degrees = std::fmod(degrees + 180.0, 360.0);
  if (degrees < 0) degrees += 360.0;
  return degrees - 180.0;
}

// Scale the reference robot's gains to a robot that moves speedRatio times as
// far per millivolt, so it reacts about the same way.
PIDGains scaleGains(PIDGains ref, double speedRatio) {
  if (!(speedRatio > 0)) return ref;
  return {ref.kP / speedRatio, ref.kI / speedRatio, ref.kD / speedRatio};
}

// Average of the motors that are plugged in (unplugged motors read infinity).
double averageFinite(const std::vector<double>& values, int& count) {
  double total = 0.0;
  for (double v : values) {
    if (std::isfinite(v)) {
      total += v;
      count++;
    }
  }
  return total;
}

}  // namespace

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------

Drive::Drive(const DriveConfig& config)
    : config_(config),
      left_(config.left_ports, config.cartridge),
      right_(config.right_ports, config.cartridge),
      imu_(config.imu_port) {
  left_.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
  right_.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
  left_.set_brake_mode_all(pros::MotorBrake::coast);
  right_.set_brake_mode_all(pros::MotorBrake::coast);

  driveGains = defaultDriveGains();
  turnGains = defaultTurnGains();
  headingGains = defaultHeadingGains();
}

void Drive::initialize() {
  startScreen();
  pros::lcd::print(0, "mechlib: calibrating IMU, keep still");

  imu_.reset(true);
  // reset() can return early if the IMU was already calibrating, so also wait here.
  for (int t = 0; imu_.is_calibrating() && t < 3000; t += 10) pros::delay(10);
  resetHeading();

  bool loaded = loadGains();
  if (!imuWorking()) {
    pros::lcd::print(0, "mechlib: IMU NOT FOUND on port %d", config_.imu_port);
  } else if (loaded) {
    pros::lcd::print(0, "mechlib ready (saved gains loaded)");
  } else {
    pros::lcd::print(0, "mechlib ready (default gains)");
  }

  // Live readout on the brain screen.
  pros::Task screen([this] {
    while (true) {
      pros::lcd::print(1, "Distance: %.1f in", getInches());
      pros::lcd::print(2, "Rotation: %.1f deg", getRotation());
      pros::delay(50);
    }
  });
}

// ----------------------------------------------------------------------------
// Driver control
// ----------------------------------------------------------------------------

void Drive::driverControl(pros::Controller& controller) {
  useBrake(false);

  int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  // Ignore tiny stick movements so the robot doesn't creep.
  const int DEADBAND = 5;
  if (std::abs(leftY) < DEADBAND) leftY = 0;
  if (std::abs(rightY) < DEADBAND) rightY = 0;
  if (std::abs(rightX) < DEADBAND) rightX = 0;

  if (config_.control == ControlMode::TANK) {
    left_.move(leftY);
    right_.move(rightY);
  } else {
    left_.move(leftY + rightX);  // PROS caps each side at 127
    right_.move(leftY - rightX);
  }

  // Keep the target heading where the driver left it, so an autonomous move
  // started from driver control (for testing) holds the current heading.
  double rotation = imu_.get_rotation();
  if (std::isfinite(rotation)) targetHeading_ = rotation;
}

// ----------------------------------------------------------------------------
// Autonomous moves
// ----------------------------------------------------------------------------

MoveResult Drive::move(double inches, int timeoutMs, int maxSpeed) {
  MoveResult result;
  useBrake(true);

  double maxVoltage = std::clamp(maxSpeed, 1, 100) * 120.0;
  // I needs to start working before P alone gets too weak to move the robot
  // (about 1500 mV), so the zone grows if kP is small.
  double integralZone = std::max(6.0, driveGains.kP > 0 ? 1500.0 / driveGains.kP : 6.0);
  PID drivePid(driveGains, integralZone, 3000.0);
  PID headingPid(headingGains, 5.0, 2000.0);

  left_.tare_position_all();
  right_.tare_position_all();
  bool useImu = imuWorking();
  double holdHeading = targetHeading_;
  double direction = inches >= 0 ? 1.0 : -1.0;

  double lastPosition = 0.0;
  double lastOutput = 0.0;
  int settledMs = 0;
  int stoppedMs = 0;
  int elapsed = 0;
  result.timedOut = true;

  for (; elapsed < timeoutMs; elapsed += LOOP_MS) {
    double position = getInches();
    double error = inches - position;
    double speed = (position - lastPosition) / LOOP_DT;
    lastPosition = position;
    result.overshoot = std::max(result.overshoot, (position - inches) * direction);

    // Drive PID: how hard to drive forward or back.
    double output = std::clamp(drivePid.update(error, LOOP_DT), -maxVoltage, maxVoltage);
    if (std::fabs(output) > std::fabs(lastOutput)) {  // slew: only limit speeding up
      output = std::clamp(output, lastOutput - SLEW_MV, lastOutput + SLEW_MV);
    }
    lastOutput = output;

    // Heading PID: steer back to the target heading.
    // IMU rotation goes UP turning right, so a positive error means steer right.
    double turn = 0.0;
    double rotation = useImu ? imu_.get_rotation() : 0.0;
    if (!std::isfinite(rotation)) useImu = false;  // IMU lost: finish on encoders only
    if (useImu) {
      double headingError = holdHeading - rotation;
      result.headingDrift = std::max(result.headingDrift, std::fabs(headingError));
      turn = headingPid.update(headingError, LOOP_DT);
    }

    setVoltage(output + turn, output - turn);

    bool slow = std::fabs(speed) < DRIVE_SETTLE_SPEED;
    settledMs = (slow && std::fabs(error) < DRIVE_SETTLE_INCHES) ? settledMs + LOOP_MS : 0;
    stoppedMs = (slow && std::fabs(error) < 2 * DRIVE_SETTLE_INCHES) ? stoppedMs + LOOP_MS : 0;
    if (settledMs >= SETTLE_MS || stoppedMs >= STOPPED_MS) {
      result.timedOut = false;
      break;
    }
    pros::delay(LOOP_MS);
  }

  stop();
  result.timeMs = elapsed;
  result.finalError = (inches - getInches()) * direction;
  return result;
}

MoveResult Drive::turn(double degrees, int timeoutMs, int maxSpeed) {
  return turnToRotation(targetHeading_ + degrees, timeoutMs, maxSpeed);
}

MoveResult Drive::turnTo(double heading, int timeoutMs, int maxSpeed) {
  return turnToRotation(targetHeading_ + wrapAngle(heading - targetHeading_), timeoutMs, maxSpeed);
}

MoveResult Drive::turnToRotation(double target, int timeoutMs, int maxSpeed) {
  MoveResult result;
  result.timedOut = true;
  useBrake(true);

  if (!imuWorking()) {
    pros::lcd::print(0, "mechlib: IMU NOT WORKING, turn skipped");
    return result;
  }

  double maxVoltage = std::clamp(maxSpeed, 1, 100) * 120.0;
  // Same idea as in move(): I must kick in before P gets too weak to turn the robot.
  double integralZone = std::max(20.0, turnGains.kP > 0 ? 2400.0 / turnGains.kP : 20.0);
  PID turnPid(turnGains, integralZone, 3000.0);

  double start = imu_.get_rotation();
  double direction = target >= start ? 1.0 : -1.0;
  double lastAngle = start;
  int settledMs = 0;
  int stoppedMs = 0;
  int elapsed = 0;

  for (; elapsed < timeoutMs; elapsed += LOOP_MS) {
    double angle = imu_.get_rotation();
    if (!std::isfinite(angle)) {  // IMU unplugged mid-turn: stop, don't spin
      pros::lcd::print(0, "mechlib: IMU LOST, turn stopped");
      break;
    }
    double error = target - angle;
    double turnSpeed = (angle - lastAngle) / LOOP_DT;
    lastAngle = angle;
    result.overshoot = std::max(result.overshoot, (angle - target) * direction);

    // Positive output = turn right: left side forward, right side back.
    double output = std::clamp(turnPid.update(error, LOOP_DT), -maxVoltage, maxVoltage);
    setVoltage(output, -output);

    bool slow = std::fabs(turnSpeed) < TURN_SETTLE_SPEED;
    settledMs = (slow && std::fabs(error) < TURN_SETTLE_DEGREES) ? settledMs + LOOP_MS : 0;
    stoppedMs = (slow && std::fabs(error) < 2 * TURN_SETTLE_DEGREES) ? stoppedMs + LOOP_MS : 0;
    if (settledMs >= SETTLE_MS || stoppedMs >= STOPPED_MS) {
      result.timedOut = false;
      break;
    }
    pros::delay(LOOP_MS);
  }

  stop();
  targetHeading_ = target;
  result.timeMs = elapsed;
  double finalAngle = imu_.get_rotation();
  if (std::isfinite(finalAngle)) result.finalError = (target - finalAngle) * direction;
  return result;
}

void Drive::resetHeading() {
  imu_.tare_rotation();
  targetHeading_ = 0.0;
}

// ----------------------------------------------------------------------------
// Default gains
// ----------------------------------------------------------------------------

double Drive::topSpeed() const {
  double rpm = 600.0;
  if (config_.cartridge == pros::MotorGearset::green) rpm = 200.0;
  if (config_.cartridge == pros::MotorGearset::red) rpm = 100.0;
  return rpm * config_.gear_ratio / 60.0 * M_PI * config_.wheel_diameter;
}

PIDGains Drive::defaultDriveGains() const {
  return scaleGains(REF_DRIVE_GAINS, topSpeed() / REF_WHEEL_SPEED);
}

PIDGains Drive::defaultTurnGains() const {
  return scaleGains(REF_TURN_GAINS, turnSpeed() / REF_TURN_SPEED);
}

PIDGains Drive::defaultHeadingGains() const {
  // Keeping straight is a small turn, so it scales the same way as turning.
  return scaleGains(REF_HEADING_GAINS, turnSpeed() / REF_TURN_SPEED);
}

double Drive::turnSpeed() const {
  // Both sides at full speed in opposite directions.
  if (!(config_.track_width > 0)) return 0.0;
  return 2.0 * topSpeed() / config_.track_width * 180.0 / M_PI;
}

// ----------------------------------------------------------------------------
// Saving gains to the SD card
// ----------------------------------------------------------------------------

bool Drive::saveGains() {
  if (!pros::usd::is_installed()) return false;
  FILE* file = std::fopen(GAINS_FILE, "w");
  if (!file) return false;
  std::fprintf(file, "%s\n%f %f %f\n%f %f %f\n%f %f %f\n", GAINS_HEADER,
               driveGains.kP, driveGains.kI, driveGains.kD,
               turnGains.kP, turnGains.kI, turnGains.kD,
               headingGains.kP, headingGains.kI, headingGains.kD);
  std::fclose(file);
  return true;
}

bool Drive::loadGains() {
  if (!pros::usd::is_installed()) return false;
  FILE* file = std::fopen(GAINS_FILE, "r");
  if (!file) return false;

  char header[32] = {0};
  double v[9];
  int count = std::fscanf(file, "%31s %lf %lf %lf %lf %lf %lf %lf %lf %lf", header,
                          &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8]);
  std::fclose(file);

  if (count != 10 || std::strcmp(header, GAINS_HEADER) != 0) return false;
  for (double value : v) {
    if (!std::isfinite(value) || value < 0) return false;
  }
  driveGains = {v[0], v[1], v[2]};
  turnGains = {v[3], v[4], v[5]};
  headingGains = {v[6], v[7], v[8]};
  return true;
}

// ----------------------------------------------------------------------------
// Setup checker
// ----------------------------------------------------------------------------

void Drive::checkSetup() {
  std::vector<std::string> problems;
  useBrake(false);
  for (int line = 3; line <= 6; line++) pros::lcd::print(line, "");
  pros::lcd::print(0, "Setup check: robot will turn a little");

  // 1. Is everything plugged in?
  for (const auto* ports : {&config_.left_ports, &config_.right_ports}) {
    for (std::int8_t port : *ports) {
      pros::Motor motor(port);
      if (!std::isfinite(motor.get_position())) {
        problems.push_back("Port " + std::to_string(std::abs(port)) + ": no motor found");
      }
    }
  }
  bool imuOk = imuWorking();
  if (!imuOk) problems.push_back("IMU port " + std::to_string(config_.imu_port) + ": not found");

  // 2. Spin one side forward for a moment, and measure how far each motor and
  // the robot turned. The motor at flipIndex (if any) is sent backward instead.
  struct Spin {
    std::vector<double> motorDegrees;
    double robotDegrees;
  };
  auto spin = [&](const std::vector<std::int8_t>& ports, int flipIndex) {
    std::vector<pros::Motor> motors;
    std::vector<double> before;
    for (std::int8_t port : ports) {
      motors.emplace_back(port);
      before.push_back(motors.back().get_position());
    }
    double rotationBefore = imu_.get_rotation();
    for (size_t i = 0; i < motors.size(); i++) motors[i].move_voltage(static_cast<int>(i) == flipIndex ? -6000 : 6000);
    pros::delay(400);
    for (auto& motor : motors) motor.move_voltage(0);
    pros::delay(400);
    Spin result;
    for (size_t i = 0; i < motors.size(); i++) result.motorDegrees.push_back(motors[i].get_position() - before[i]);
    result.robotDegrees = imu_.get_rotation() - rotationBefore;
    return result;
  };
  auto robotTurned = [&](const Spin& s) { return imuOk && std::isfinite(s.robotDegrees) && std::fabs(s.robotDegrees) >= 3.0; };

  int motorCount = 0;
  std::vector<std::string> reversed;  // "Port 3: change -3 to 3" lines
  auto checkSide = [&](const std::vector<std::int8_t>& ports, bool isLeft) {
    std::string side = isLeft ? "Left" : "Right";
    Spin s = spin(ports, -1);
    // If the robot didn't turn, the motors may be fighting each other. Try again
    // with each motor sent the other way, until one try gets the side moving.
    for (int i = 0; imuOk && !robotTurned(s) && i < static_cast<int>(ports.size()); i++) s = spin(ports, i);
    if (imuOk && !robotTurned(s)) {
      problems.push_back(side + " side won't move. Jammed?");
      return;
    }

    // Left side forward turns the robot right (rotation goes up).
    // Right side forward turns the robot left (rotation goes down).
    bool sideWentForward = isLeft ? s.robotDegrees > 0 : s.robotDegrees < 0;

    // Without the IMU, the best we can do is find motors that disagree with the rest.
    int positives = 0;
    int negatives = 0;
    for (double d : s.motorDegrees) {
      if (!std::isfinite(d) || std::fabs(d) < 5.0) continue;
      if (d > 0) positives++;
      else negatives++;
    }
    if (!imuOk && positives == negatives && positives > 0) {
      problems.push_back(side + " side: can't tell without IMU");
      return;
    }

    for (size_t i = 0; i < ports.size(); i++) {
      double d = s.motorDegrees[i];
      if (!std::isfinite(d)) continue;  // unplugged, already reported
      motorCount++;
      std::string port = std::to_string(std::abs(ports[i]));
      if (std::fabs(d) < 5.0) {
        problems.push_back("Port " + port + ": didn't turn");
        continue;
      }
      // Every motor on a side turns together. A motor with the right sign
      // reads forward (positive) when its side really went forward.
      bool wrong = imuOk ? (d > 0) != sideWentForward : (d > 0 ? positives < negatives : negatives < positives);
      if (wrong) reversed.push_back("Port " + port + ": change " + std::to_string(ports[i]) + " to " + std::to_string(-ports[i]));
    }
  };

  checkSide(config_.left_ports, true);
  pros::delay(300);
  checkSide(config_.right_ports, false);
  stop();

  if (motorCount > 0 && static_cast<int>(reversed.size()) == motorCount) {
    // Every motor looks wrong. Every sign is wrong, the left and right lists are
    // swapped, or the IMU is upside down. Driving with the sticks tells them apart.
    problems.push_back("All motors look reversed. Drive & see:");
    problems.push_back("Drives backward? Flip every port sign");
    problems.push_back("Turns wrong way? Swap left/right lists");
    problems.push_back("Both fine? IMU is upside down");
  } else {
    problems.insert(problems.end(), reversed.begin(), reversed.end());
  }

  // 3. Report.
  if (problems.empty()) {
    pros::lcd::print(0, "Setup check: everything looks good!");
  } else {
    pros::lcd::print(0, "Setup check: fix the lines below");
    for (size_t i = 0; i < problems.size(); i++) {
      std::printf("mechlib setup: %s\n", problems[i].c_str());  // full list in the terminal
      if (i < 4) pros::lcd::print(3 + i, "%s", problems[i].c_str());
    }
    if (problems.size() > 4) pros::lcd::print(6, "+%d more, see PROS terminal", (int)problems.size() - 3);
  }
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

double Drive::getInches() {
  int count = 0;
  double total = averageFinite(left_.get_position_all(), count);
  total += averageFinite(right_.get_position_all(), count);
  if (count == 0) return 0.0;
  double motorDegrees = total / count;
  return motorDegrees / 360.0 * config_.gear_ratio * M_PI * config_.wheel_diameter;
}

double Drive::getRotation() {
  return imu_.get_rotation();
}

bool Drive::imuWorking() {
  return std::isfinite(imu_.get_rotation());
}

void Drive::setVoltage(double left, double right) {
  // If one side asks for more than 12 V, scale both down together so the
  // robot still steers the right amount.
  double biggest = std::max(std::fabs(left), std::fabs(right));
  if (biggest > 12000.0) {
    left *= 12000.0 / biggest;
    right *= 12000.0 / biggest;
  }
  left_.move_voltage(static_cast<int>(left));
  right_.move_voltage(static_cast<int>(right));
}

void Drive::stop() {
  left_.move_voltage(0);
  right_.move_voltage(0);
  left_.brake();
  right_.brake();
}

void Drive::useBrake(bool brake) {
  if (brake == braking_) return;
  pros::MotorBrake mode = brake ? pros::MotorBrake::brake : pros::MotorBrake::coast;
  left_.set_brake_mode_all(mode);
  right_.set_brake_mode_all(mode);
  braking_ = brake;
}

}  // namespace mechlib
