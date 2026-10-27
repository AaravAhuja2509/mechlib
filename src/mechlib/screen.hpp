#pragma once

// Brain screen helpers used inside mechlib. Not part of the public API.

#include "api.h"

namespace mechlib {

// Brain screen text (pros::lcd) comes from the liblvgl template, which every
// new PROS project includes. If a project doesn't have it, these functions are
// missing and calling them would crash the program, so mechlib checks first and
// skips the screen. pros::lcd::print is always safe: without liblvgl it does nothing.
inline bool screenAvailable() {
  return &pros::lcd::initialize != nullptr && &pros::lcd::is_initialized != nullptr;
}

// Turn on the brain screen if nothing has yet. Returns false if there is no screen.
inline bool startScreen() {
  if (!screenAvailable()) return false;
  if (!pros::lcd::is_initialized()) pros::lcd::initialize();
  return true;
}

}  // namespace mechlib
