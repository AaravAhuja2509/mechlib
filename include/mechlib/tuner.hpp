#pragma once

#include "mechlib/drive.hpp"

namespace mechlib {

// Guided PID tuner on the controller. Call it from opcontrol() (for example
// when a button is pressed). It takes over the controller until you press B.
//
//   A           run a test move with the current value
//   UP / DOWN   raise / lower the value shown
//   RIGHT/LEFT  next / previous step
//   X           put this value back to its default
//   Y           save all gains to the SD card (they load automatically next time)
//   B           exit the tuner
//
// Drive tests go 24 inches and turn tests go 180 degrees, alternating
// direction so the robot stays in about the same place. After each test the
// controller shows the overshoot (O), how far from the target it stopped (E)
// and how long it took, plus a hint telling you what to change next. The
// brain screen shows the same numbers in words.
// The sticks still drive while the tuner is open, so you can line the robot up.
void runTuner(Drive& drive, pros::Controller& controller);

}  // namespace mechlib
