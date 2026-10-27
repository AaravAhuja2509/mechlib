#pragma once

#include <cstdint>
#include <vector>

#include "api.h"
#include "mechlib/pid.hpp"

namespace mechlib {

enum class ControlMode {
  TANK,    // left stick = left side, right stick = right side
  ARCADE,  // left stick up/down = forward/back, right stick left/right = turn
};

// Everything mechlib needs to know about your robot.
// Fill these in when you create your Drive. Keep the fields in this order.
struct DriveConfig {
  std::vector<std::int8_t> left_ports;   // negative port = reversed motor
  std::vector<std::int8_t> right_ports;
  std::int8_t imu_port;
  double wheel_diameter;                 // inches (2.75, 3.25, 4.0 ...)
  double gear_ratio = 1.0;               // wheel turns per motor turn. 36:48 = 0.75, direct = 1.0
  pros::MotorGearset cartridge = pros::MotorGearset::blue;  // blue 600, green 200, red 100 RPM
  double track_width = 12.0;             // inches, center of left wheels to center of right wheels
  ControlMode control = ControlMode::ARCADE;
};

// What happened during one move or turn. The tuner shows this on the controller.
struct MoveResult {
  double overshoot = 0.0;    // how far it went past the target (inches or degrees)
  double finalError = 0.0;   // how far from the target it stopped (+ = short, - = past)
  double headingDrift = 0.0; // biggest heading error while driving (degrees, moves only)
  int timeMs = 0;            // how long the move took
  bool timedOut = false;     // true if it gave up before settling
};

class Drive {
 public:
  explicit Drive(const DriveConfig& config);

  // ---- Call these from your competition functions ----

  // Call once in initialize(). Calibrates the IMU (keep the robot still for
  // about 2 seconds) and loads any gains saved by the tuner on the SD card.
  void initialize();

  // Call every loop in opcontrol(). Drives with the sticks (tank or arcade).
  void driverControl(pros::Controller& controller);

  // ---- Autonomous moves ----

  // Drive forward (negative = backward) in inches, keeping the robot straight.
  // maxSpeed is a percent, 1 to 100.
  MoveResult move(double inches, int timeoutMs = 3000, int maxSpeed = 100);

  // Turn by a number of degrees from the current target heading.
  // Positive = right (clockwise), negative = left.
  MoveResult turn(double degrees, int timeoutMs = 2000, int maxSpeed = 100);

  // Turn to face an absolute heading, 0 to 360, measured from where the robot
  // faced at the last resetHeading(). Always turns the short way.
  MoveResult turnTo(double heading, int timeoutMs = 2000, int maxSpeed = 100);

  // Make the way the robot faces right now heading 0. Call at the start of autonomous().
  void resetHeading();

  // ---- Tools ----

  // Spins each side and tells you which ports are unplugged or have the
  // wrong sign. The robot turns a little in place, so give it room.
  void checkSetup();

  // Save / load the current gains to the SD card.
  bool saveGains();
  bool loadGains();

  // Starting gains worked out from your wheel size, gearing and track width.
  PIDGains defaultDriveGains() const;
  PIDGains defaultTurnGains() const;
  PIDGains defaultHeadingGains() const;

  // Live sensor values.
  double getInches();   // distance driven since the last move started
  double getRotation(); // IMU rotation in degrees (doesn't wrap at 360)
  bool imuWorking();

  // Top speed in inches per second at full power, worked out from the
  // cartridge, gear ratio and wheel size.
  double topSpeed() const;

  // The gains in use. You can set these in code, or change them with the tuner.
  PIDGains driveGains;    // drive distance, mV per inch
  PIDGains turnGains;     // turning, mV per degree
  PIDGains headingGains;  // keeping straight while driving, mV per degree

 private:
  MoveResult turnToRotation(double target, int timeoutMs, int maxSpeed);
  void setVoltage(double left, double right);
  void stop();
  void useBrake(bool brake);
  double turnSpeed() const;  // degrees per second at full power

  DriveConfig config_;
  pros::MotorGroup left_;
  pros::MotorGroup right_;
  pros::Imu imu_;
  double targetHeading_ = 0.0;   // where the robot should be pointing (IMU rotation)
  bool braking_ = false;
};

}  // namespace mechlib
