#include "mechlib/tuner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace mechlib {

namespace {

enum class Kind { DRIVE, TURN, HEADING };
enum class Term { P, I, D };

struct Step {
  const char* label;  // 8 characters max, shown on the controller
  PIDGains* gains;
  Term term;
  Kind kind;
  double defaultValue;
};

double& valueOf(const Step& step) {
  if (step.term == Term::P) return step.gains->kP;
  if (step.term == Term::I) return step.gains->kI;
  return step.gains->kD;
}

// How far from the target is close enough, how much overshoot is way too
// much, and how long is too slow, for one test.
struct Limits {
  double tolerance, tooFar, slowSeconds;
};

Limits limitsFor(Kind kind, double topSpeed) {
  if (kind == Kind::TURN) return {1.0, 2.5, 1.8};  // degrees and seconds, for the 180 degree test turn
  // Inches, for the 24 inch test drive. A slower robot needs longer to drive 24 inches.
  double slowSeconds = 1.2 + (topSpeed > 0 ? 24.0 / topSpeed : 0.6);
  return {0.5, 1.0, slowSeconds};
}

// The hint shown on the controller after a test (15 characters max).
// The plan for drives and turns: raise kP until the robot gets there quickly,
// raise kD until it stops going past the target, then set kI so it finishes
// the last bit without creeping past.
std::string hintFor(const Step& step, const MoveResult& r, const Limits& lim) {
  if (step.kind == Kind::HEADING) {
    bool straight = r.headingDrift <= 2.0;
    if (step.term == Term::P) return straight ? "Straight! Next>" : "Curving: UP kP";
    return straight ? "Good! Next >" : "Wiggles? UP kD";
  }

  // Going past the target is fine if the robot still stops close enough.
  bool overshot = r.overshoot > lim.tolerance;
  bool stoppedShort = r.finalError > lim.tolerance;
  bool slow = r.timeMs / 1000.0 > lim.slowSeconds;
  switch (step.term) {
    case Term::P:
      if (r.overshoot > lim.tooFar) return "Too far: DN kP";
      if (stoppedShort) return "Short: UP kP";
      if (slow) return "Slow: UP kP";
      return "Good! Next >";
    case Term::D:
      if (overshot) return "Overshot: UP kD";
      if (slow) return "Sluggish: DN kD";
      return "Good! Next >";
    case Term::I:
      if (overshot) return "Overshot: DN kI";
      if (stoppedShort || r.timedOut) return "Short: UP kI";
      return "Good! Next >";
  }
  return "";
}

// "O0.4 E0.1 1.1s": overshoot, how far from the target it stopped, and time.
// Numbers of 10 or more drop the decimal so the line fits on the controller.
std::string resultText(const MoveResult& r) {
  auto number = [](double x) {
    char text[16];
    std::snprintf(text, sizeof(text), std::fabs(x) < 10 ? "%.1f" : "%.0f", x);
    return std::string(text);
  };
  return "O" + number(r.overshoot) + " E" + number(std::fabs(r.finalError)) + " " + number(r.timeMs / 1000.0) + "s";
}

// The controller screen is 3 lines of 15 characters, and only takes a new
// line about every 50 ms.
void showOnController(pros::Controller& controller, const std::string lines[3]) {
  for (int i = 0; i < 3; i++) {
    controller.print(i, 0, "%-15.15s", lines[i].c_str());
    pros::delay(60);
  }
}

}  // namespace

void runTuner(Drive& drive, pros::Controller& controller) {
  PIDGains driveDefault = drive.defaultDriveGains();
  PIDGains turnDefault = drive.defaultTurnGains();
  PIDGains headingDefault = drive.defaultHeadingGains();

  // The tuning order: P first, then D to stop overshoot, then I to finish.
  Step steps[] = {
      {"Drive kP", &drive.driveGains, Term::P, Kind::DRIVE, driveDefault.kP},
      {"Drive kD", &drive.driveGains, Term::D, Kind::DRIVE, driveDefault.kD},
      {"Drive kI", &drive.driveGains, Term::I, Kind::DRIVE, driveDefault.kI},
      {"Turn kP", &drive.turnGains, Term::P, Kind::TURN, turnDefault.kP},
      {"Turn kD", &drive.turnGains, Term::D, Kind::TURN, turnDefault.kD},
      {"Turn kI", &drive.turnGains, Term::I, Kind::TURN, turnDefault.kI},
      {"Head kP", &drive.headingGains, Term::P, Kind::HEADING, headingDefault.kP},
      {"Head kD", &drive.headingGains, Term::D, Kind::HEADING, headingDefault.kD},
  };
  const int STEP_COUNT = sizeof(steps) / sizeof(steps[0]);

  int current = 0;
  int testDirection = 1;  // tests go forward then back (or right then left), so the robot stays in place
  std::string resultLine = "A = test";
  std::string hintLine = "UP/DN  >=next";
  bool redraw = true;

  for (int line = 3; line <= 5; line++) pros::lcd::print(line, "");
  pros::lcd::print(6, "A test, UP/DN value, Y save, B exit");

  while (true) {
    // The sticks still drive, so you can line the robot up between tests.
    drive.driverControl(controller);

    Step& step = steps[current];
    double& value = valueOf(step);
    // Each press changes the value by 10%, and never by less than 10% of the default.
    double change = std::max(value * 0.1, step.defaultValue * 0.1);

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      value += change;
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      value = std::max(0.0, value - change);
      if (value < step.defaultValue * 0.01) value = 0.0;
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      current = (current + 1) % STEP_COUNT;
      resultLine = "A = test";
      hintLine = "UP/DN  >=next";
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      current = (current + STEP_COUNT - 1) % STEP_COUNT;
      resultLine = "A = test";
      hintLine = "UP/DN  >=next";
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      value = step.defaultValue;
      hintLine = "Back to default";
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      bool saved = drive.saveGains();
      hintLine = saved ? "Saved to SD!" : "No SD card!";
      if (saved) controller.rumble(".");
      redraw = true;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      std::string lines[3] = {"Tuner closed", "Y in tuner=save", ""};
      showOnController(controller, lines);
      for (int line = 3; line <= 6; line++) pros::lcd::print(line, "");
      return;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      std::string running[3] = {step.label, "Testing...", ""};
      showOnController(controller, running);

      drive.resetHeading();  // test from wherever the robot is facing now
      MoveResult r;
      if (step.kind == Kind::TURN) {
        // Test with a half turn: the bigger the turn, the harder it is to stop,
        // so gains that stop a 180 degree turn also stop smaller turns.
        r = drive.turn(180.0 * testDirection);
        resultLine = resultText(r);
        pros::lcd::print(4, "Over %.1f deg  Stopped %.1f deg %s", r.overshoot, std::fabs(r.finalError),
                         r.finalError >= 0 ? "short" : "past");
        pros::lcd::print(5, "Time %.2f s%s", r.timeMs / 1000.0, r.timedOut ? " TIMED OUT" : "");
      } else {
        r = drive.move(24.0 * testDirection);
        if (step.kind == Kind::HEADING) {
          char drift[16];
          std::snprintf(drift, sizeof(drift), "Drift %.1f deg", r.headingDrift);
          resultLine = drift;
        } else {
          resultLine = resultText(r);
        }
        pros::lcd::print(4, "Over %.2f in  Stopped %.2f in %s", r.overshoot, std::fabs(r.finalError),
                         r.finalError >= 0 ? "short" : "past");
        pros::lcd::print(5, "Time %.2f s%s  Drift %.1f deg", r.timeMs / 1000.0, r.timedOut ? " TIMED OUT" : "",
                         r.headingDrift);
      }
      testDirection = -testDirection;
      hintLine = hintFor(step, r, limitsFor(step.kind, drive.topSpeed()));
      if (current == STEP_COUNT - 1 && hintLine == "Good! Next >") hintLine = "Done! Y = save";
      pros::lcd::print(6, "Hint: %s", hintLine.c_str());
      redraw = true;
    }

    if (redraw) {
      char top[32];
      std::snprintf(top, sizeof(top), value < 10 ? "%-8s%7.2f" : "%-8s%7.1f", step.label, value);
      std::string lines[3] = {top, resultLine, hintLine};
      showOnController(controller, lines);
      pros::lcd::print(3, "TUNER %d/%d  %s = %.2f", current + 1, STEP_COUNT, step.label, value);
      redraw = false;
    }

    pros::delay(20);
  }
}

}  // namespace mechlib
