// mechlib simulator: runs the real library code against the fake PROS in
// mock/api.h and prints how the robot moved. See README.md in this folder.
//
//   ./mechsim motion   16 robots, 8 moves and turns each, with the default gains
//   ./mechsim setup    the setup check on 10 wiring mistakes
//   ./mechsim tuner    a "rookie" follows every tuner hint on 12 robots
//   ./mechsim          all three
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mechlib/mechlib.hpp"

using namespace mechlib;

// The brain screen functions that mock/api.h declares weak, like real PROS does.
bool pros::lcd::is_initialized() { return initialized(); }
bool pros::lcd::initialize() {
  initialized() = true;
  return true;
}

// A robot to simulate: the DriveConfig a team would write, plus the physics.
struct Robot {
  std::string name;
  DriveConfig cfg;
  SimParams p;
};

Robot makeRobot(std::string name, double wheel, double gear, pros::MotorGearset cart, double trackIn,
                double mass = 6.0, double J = 0.15, int perSide = 3) {
  Robot r;
  r.name = name;
  std::vector<std::int8_t> L, R;
  for (int i = 0; i < perSide; i++) {
    L.push_back(-(i + 1));
    R.push_back(i + 11);
  }
  r.cfg = DriveConfig{L, R, 10, wheel, gear, cart, trackIn, ControlMode::ARCADE};
  r.p.leftDir.assign(perSide, 1);
  r.p.rightDir.assign(perSide, 1);
  double rpm = cart == pros::MotorGearset::blue ? 600 : cart == pros::MotorGearset::green ? 200 : 100;
  r.p.r = wheel * 0.0254 / 2;
  r.p.gear = gear;
  r.p.track = trackIn * 0.0254;
  r.p.wfree = rpm * 2 * M_PI / 60;
  r.p.Tu = 0.7 * 600 / rpm;
  r.p.Tlim = 0.35 * 600 / rpm;
  r.p.mass = mass;
  r.p.J = J;
  return r;
}

std::unique_ptr<Drive> build(const Robot& r) {
  pros::groupCount() = 0;
  pros::groupPorts().clear();
  sim().p = r.p;
  sim().reset();
  return std::make_unique<Drive>(r.cfg);
}

// The worst results seen, for the summary at the end of a suite.
struct Tally {
  int moves = 0, timeouts = 0;
  double worstMove = 0, worstTurn = 0;  // how far from the target the robot really ended
};

// Run one move or turn, let the robot coast to a stop, and print where it really ended.
void runAndReport(Drive& d, bool turn, double target, Tally& tally) {
  sim().reset();
  d.resetHeading();
  MoveResult res = turn ? d.turn(target) : d.move(target);
  pros::delay(300);
  double end = turn ? sim().degrees() : sim().inches();
  printf("  %-4s %6.1f  %4d ms%s  overshoot %5.2f  ended at %7.2f %s\n", turn ? "turn" : "move", target, res.timeMs,
         res.timedOut ? " TIMED OUT" : "          ", res.overshoot, end, turn ? "deg" : "in");
  tally.moves++;
  if (res.timedOut) tally.timeouts++;
  double off = std::fabs(end - target);
  if (turn) tally.worstTurn = std::max(tally.worstTurn, off);
  else tally.worstMove = std::max(tally.worstMove, off);
}

void printGains(const char* label, const Drive& d) {
  printf("  %s drive {%.0f %.0f %.1f}  turn {%.1f %.1f %.1f}  heading {%.1f %.1f %.1f}\n", label, d.driveGains.kP,
         d.driveGains.kI, d.driveGains.kD, d.turnGains.kP, d.turnGains.kI, d.turnGains.kD, d.headingGains.kP,
         d.headingGains.kI, d.headingGains.kD);
}

// ----------------------------------------------------------------------------
// motion: the default gains on many different robots
// ----------------------------------------------------------------------------

void motionSuite() {
  std::vector<Robot> robots = {
      makeRobot("reference 450rpm 3.25in 11.5in", 3.25, 0.75, pros::MotorGearset::blue, 11.5),
      makeRobot("example 450rpm 3.25in 12in", 3.25, 0.75, pros::MotorGearset::blue, 12.0),
      makeRobot("360rpm 3.25in 12in", 3.25, 0.6, pros::MotorGearset::blue, 12.0),
      makeRobot("600rpm 3.25in 12in", 3.25, 1.0, pros::MotorGearset::blue, 12.0),
      makeRobot("450rpm 2.75in 11in", 2.75, 0.75, pros::MotorGearset::blue, 11.0),
      makeRobot("green 200rpm 4in 13in", 4.0, 1.0, pros::MotorGearset::green, 13.0),
      makeRobot("blue 600rpm 4in 10in (fast)", 4.0, 1.0, pros::MotorGearset::blue, 10.0),
      makeRobot("heavy 450rpm 3.25in 8kg", 3.25, 0.75, pros::MotorGearset::blue, 12.0, 8.0, 0.22),
      makeRobot("4-motor 450rpm 3.25in", 3.25, 0.75, pros::MotorGearset::blue, 12.0, 5.0, 0.12, 2),
      makeRobot("4-motor 600rpm 2.75in", 2.75, 1.0, pros::MotorGearset::blue, 11.0, 5.0, 0.12, 2),
      makeRobot("4-motor green 4in (kit bot)", 4.0, 1.0, pros::MotorGearset::green, 14.0, 5.0, 0.14, 2),
      makeRobot("8-motor 600rpm 3.25in", 3.25, 1.0, pros::MotorGearset::blue, 12.0, 7.0, 0.18, 4),
  };
  Robot r = makeRobot("example + high friction", 3.25, 0.75, pros::MotorGearset::blue, 12.0);
  r.p.linStaticMv = 1100, r.p.linKineticMv = 900, r.p.rotStaticMv = 2000, r.p.rotKineticMv = 1700;
  robots.push_back(r);
  r = makeRobot("example + low friction", 3.25, 0.75, pros::MotorGearset::blue, 12.0);
  r.p.linStaticMv = 500, r.p.linKineticMv = 400, r.p.rotStaticMv = 800, r.p.rotKineticMv = 650;
  robots.push_back(r);
  r = makeRobot("example + 10ms sensor lag", 3.25, 0.75, pros::MotorGearset::blue, 12.0);
  r.p.sensLatencyMs = 10;
  robots.push_back(r);
  r = makeRobot("example + right side 5% weak", 3.25, 0.75, pros::MotorGearset::blue, 12.0);
  r.p.sideBias = 0.95;
  robots.push_back(r);

  Tally tally;
  for (const Robot& robot : robots) {
    auto d = build(robot);
    printf("\n== %s (top speed %.0f in/s)\n", robot.name.c_str(), d->topSpeed());
    printGains("default", *d);
    for (double inches : {24.0, -24.0, 6.0, 48.0}) runAndReport(*d, false, inches, tally);
    for (double degrees : {90.0, -90.0, 180.0, 15.0}) runAndReport(*d, true, degrees, tally);
  }
  printf("\nMOTION: %d moves and turns, %d timed out. Worst: %.2f in off on a move, %.2f deg off on a turn.\n",
         tally.moves, tally.timeouts, tally.worstMove, tally.worstTurn);
}

// ----------------------------------------------------------------------------
// setup: the setup check on wiring mistakes
// ----------------------------------------------------------------------------

void setupCase(const char* title, Robot r, std::function<void(SimParams&)> mistake) {
  mistake(r.p);
  auto d = build(r);
  for (auto& line : screens().lcd) line.clear();
  printf("\n-- %s\n", title);
  fflush(stdout);
  d->checkSetup();
  for (int i = 0; i < 8; i++) {
    if (!screens().lcd[i].empty() && i != 1 && i != 2) printf("    brain line %d | %s\n", i, screens().lcd[i].c_str());
  }
}

void setupSuite() {
  Robot six = makeRobot("example", 3.25, 0.75, pros::MotorGearset::blue, 12.0);
  Robot four = makeRobot("4-motor", 3.25, 0.75, pros::MotorGearset::blue, 12.0, 5.0, 0.12, 2);
  // leftDir/rightDir: 1 = the sign in the config is right, -1 = it's wrong, 0 = unplugged.
  setupCase("everything right", six, [](SimParams&) {});
  setupCase("left motor 2 reversed", six, [](SimParams& p) { p.leftDir = {1, -1, 1}; });
  setupCase("whole right side reversed", six, [](SimParams& p) { p.rightDir = {-1, -1, -1}; });
  setupCase("every motor reversed", six, [](SimParams& p) { p.leftDir = {-1, -1, -1}, p.rightDir = {-1, -1, -1}; });
  setupCase("left and right lists swapped", six, [](SimParams& p) { p.swapSides = true; });
  setupCase("right motor 12 unplugged", six, [](SimParams& p) { p.rightDir = {1, 0, 1}; });
  setupCase("no IMU, left motor 2 reversed", six, [](SimParams& p) { p.imuPresent = false, p.leftDir = {1, -1, 1}; });
  setupCase("IMU upside down", six, [](SimParams& p) { p.imuSign = -1; });
  setupCase("4-motor, left motor 2 reversed (the left side fights itself)", four,
            [](SimParams& p) { p.leftDir = {1, -1}; });
  setupCase("4-motor, everything right", four, [](SimParams&) {});
}

// ----------------------------------------------------------------------------
// tuner: a rookie who does exactly what each hint says
// ----------------------------------------------------------------------------

struct Rookie {
  std::string label;
  int testsThisStep = 0, totalTests = 0, presses = 0, gaveUp = 0;
  bool done = false;
  long seen = -1;
  std::vector<std::string> log;

  // Called whenever the tuner checks the buttons and no press is queued.
  void act() {
    if (screens().version == seen) return;  // wait for the screen to show the last press
    seen = screens().version;
    std::string top = screens().ctrl[0], result = screens().ctrl[1], hint = screens().ctrl[2];
    std::string step = top.substr(0, 8);
    if (step != label) label = step, testsThisStep = 0;
    auto press = [&](std::initializer_list<int> buttons) {
      for (int b : buttons) pressQueue().push_back(b);
      presses += buttons.size();
    };
    bool tested = !(result == "A = test" || result.empty());
    if (tested && std::getenv("ROOKIE_VERBOSE")) printf("      [%s | %s | %s]\n", top.c_str(), result.c_str(), hint.c_str());

    if (++presses > 2000 || done) return press({pros::E_CONTROLLER_DIGITAL_B});
    if (hint == "Saved to SD!" || hint == "No SD card!") {
      log.push_back("  " + hint);
      done = true;
      return press({pros::E_CONTROLLER_DIGITAL_B});
    }
    if (hint == "Done! Y = save") {
      log.push_back("  " + top + " | " + result + " | " + hint);
      return press({pros::E_CONTROLLER_DIGITAL_Y});
    }
    if (!tested) {
      testsThisStep++, totalTests++;
      return press({pros::E_CONTROLLER_DIGITAL_A});
    }
    if (testsThisStep >= 15) {  // a real rookie would give up on this step
      gaveUp++;
      log.push_back("  " + top + " | " + result + " | " + hint + "   <- GAVE UP");
      return press({pros::E_CONTROLLER_DIGITAL_RIGHT});
    }
    if (hint.rfind("Good!", 0) == 0 || hint.rfind("Straight!", 0) == 0) {
      log.push_back("  " + top + " | " + result + " | " + hint + "  (" + std::to_string(testsThisStep) + " tests)");
      return press({pros::E_CONTROLLER_DIGITAL_RIGHT});
    }
    testsThisStep++, totalTests++;
    if (hint.find("UP k") != std::string::npos) press({pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_A});
    else if (hint.find("DN k") != std::string::npos) press({pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_A});
    else press({pros::E_CONTROLLER_DIGITAL_A});
  }
};

struct TunerTally {
  int tests = 0, gaveUp = 0;
  Tally after;
};

// Start from the default gains times gainScale, tune, then try the tuned gains
// (loaded back from the SD card) on moves and turns of other sizes.
void tunerCase(const char* title, Robot r, double gainScale, TunerTally& total) {
  auto d = build(r);
  auto scale = [&](PIDGains g) { return PIDGains{g.kP * gainScale, g.kI * gainScale, g.kD * gainScale}; };
  d->driveGains = scale(d->driveGains);
  d->turnGains = scale(d->turnGains);
  d->headingGains = scale(d->headingGains);
  printf("\n-- %s (starting gains x%.2f)\n", title, gainScale);
  printGains("start", *d);

  Rookie rookie;
  pressQueue().clear();
  pressPolicy() = [&]() { rookie.act(); };
  int startMs = sim().t;
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  mechlib::runTuner(*d, controller);
  pressPolicy() = nullptr;
  for (auto& line : rookie.log) printf("%s\n", line.c_str());
  printf("  %d tests, %.0f s of tuning\n", rookie.totalTests, (sim().t - startMs) / 1000.0);
  total.tests += rookie.totalTests;
  total.gaveUp += rookie.gaveUp;

  auto fresh = build(r);
  bool loaded = fresh->loadGains();
  printGains(loaded ? "tuned (loaded from SD)" : "SD LOAD FAILED, defaults", *fresh);
  for (double inches : {24.0, 48.0, 6.0}) runAndReport(*fresh, false, inches, total.after);
  for (double degrees : {90.0, 180.0, 15.0}) runAndReport(*fresh, true, degrees, total.after);
  std::remove((usdDir() + "/mechlib_gains.txt").c_str());
}

void tunerSuite() {
  using pros::MotorGearset;
  TunerTally total;
  Robot example = makeRobot("example", 3.25, 0.75, MotorGearset::blue, 12.0);
  tunerCase("example robot", example, 1.0, total);
  tunerCase("example robot, gains far too soft", example, 0.4, total);
  tunerCase("example robot, gains far too strong", example, 2.5, total);
  tunerCase("4-motor 600rpm 2.75in", makeRobot("", 2.75, 1.0, MotorGearset::blue, 11.0, 5.0, 0.12, 2), 1.0, total);
  tunerCase("fast 600rpm 4in", makeRobot("", 4.0, 1.0, MotorGearset::blue, 10.0), 1.0, total);
  Robot heavy = makeRobot("", 3.25, 0.75, MotorGearset::blue, 12.0, 8.0, 0.22);
  heavy.p.linStaticMv = 1100, heavy.p.linKineticMv = 900, heavy.p.rotStaticMv = 2000, heavy.p.rotKineticMv = 1700;
  heavy.p.sensLatencyMs = 10;
  tunerCase("heavy, high friction, 10ms sensor lag", heavy, 1.0, total);
  tunerCase("kit bot 4-motor green 4in", makeRobot("", 4.0, 1.0, MotorGearset::green, 14.0, 5.0, 0.14, 2), 1.0, total);
  tunerCase("8-motor 600rpm 3.25in", makeRobot("", 3.25, 1.0, MotorGearset::blue, 12.0, 7.0, 0.18, 4), 1.0, total);
  Robot slippy = example;
  slippy.p.linStaticMv = 500, slippy.p.linKineticMv = 400, slippy.p.rotStaticMv = 800, slippy.p.rotKineticMv = 650;
  tunerCase("example + low friction", slippy, 1.0, total);
  tunerCase("4-motor 450rpm, gains x0.6", makeRobot("", 3.25, 0.75, MotorGearset::blue, 12.0, 5.0, 0.12, 2), 0.6, total);
  tunerCase("360rpm 3.25in, gains x1.6", makeRobot("", 3.25, 0.6, MotorGearset::blue, 12.0), 1.6, total);
  tunerCase("green 200rpm 4in, gains x0.5", makeRobot("", 4.0, 1.0, MotorGearset::green, 13.0), 0.5, total);
  printf("\nTUNER: 12 robots, %d tests, gave up on %d steps. After tuning: %d of %d moves and turns timed out. "
         "Worst: %.2f in off on a move, %.2f deg off on a turn.\n",
         total.tests, total.gaveUp, total.after.timeouts, total.after.moves, total.after.worstMove,
         total.after.worstTurn);
}

int main(int argc, char** argv) {
  std::string only = argc > 1 ? argv[1] : "all";
  std::filesystem::create_directories(usdDir());  // the fake SD card
  if (only == "all" || only == "motion") motionSuite();
  if (only == "all" || only == "setup") setupSuite();
  if (only == "all" || only == "tuner") tunerSuite();
  return 0;
}
