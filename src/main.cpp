#include "main.h"

// ============================================================================
// 1. Describe your robot. This is the only part you have to change.
// ============================================================================

mechlib::Drive drive({
    .left_ports = {-1, -2, -3},   // negative = reversed. Not sure? Press Y to run the setup check
    .right_ports = {4, 5, 6},
    .imu_port = 10,
    .wheel_diameter = 3.25,       // inches
    .gear_ratio = 0.75,           // wheel turns per motor turn. 36:48 = 0.75, direct drive = 1.0
    .cartridge = pros::MotorGearset::blue,    // blue 600 RPM, green 200, red 100
    .track_width = 12.0,          // inches, from the middle of the left wheels to the middle of the right
    .control = mechlib::ControlMode::ARCADE,  // or TANK
});

pros::Controller master(pros::E_CONTROLLER_MASTER);

// ============================================================================
// 2. Write your autonomous routines.
// ============================================================================

void square() {
  for (int i = 0; i < 4; i++) {
    drive.move(24);  // forward 24 inches
    drive.turn(90);  // turn right 90 degrees (negative turns left)
  }
}

void driveAndBack() {
  drive.move(36);
  drive.move(-36);  // negative inches drive backward
}

void slowAndTurnTo() {
  drive.move(24, 2000, 50);  // 24 inches, give up after 2 seconds, at 50% speed
  drive.turnTo(180);         // face the opposite way from where autonomous started
  drive.move(24);
}

// ============================================================================
// 3. The PROS competition functions.
// ============================================================================

void initialize() {
  drive.initialize();  // calibrates the IMU: keep the robot still for 2 seconds

  // Pick a routine with the left and right buttons under the brain screen.
  mechlib::auton::add("Square", square);
  mechlib::auton::add("Drive and back", driveAndBack);
  mechlib::auton::add("Slow, then turnTo", slowAndTurnTo);
  mechlib::auton::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  drive.resetHeading();   // the way the robot faces now becomes heading 0
  mechlib::auton::run();  // runs the routine picked on the brain screen
}

void opcontrol() {
  while (true) {
    drive.driverControl(master);

    // Practice tools. They only work when the robot is NOT connected to a
    // field controller, so they can't go off by accident during a match.
    if (!pros::competition::is_connected()) {
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) drive.checkSetup();
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) mechlib::runTuner(drive, master);
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) autonomous();
    }

    pros::delay(10);
  }
}
