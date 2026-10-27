#pragma once

#include <functional>
#include <string>

namespace mechlib {
namespace auton {

// Add an autonomous routine to the selector. Call this in initialize().
//   mechlib::auton::add("Left side", leftSide);
void add(const std::string& name, std::function<void()> routine);

// Show the selector on the brain screen. Call this in initialize(), after add().
// Use the LEFT and RIGHT buttons under the brain screen to pick a routine.
// The choice is saved to the SD card (if there is one), so it survives a restart.
void initialize();

// Run the selected routine. Call this in autonomous().
void run();

}  // namespace auton
}  // namespace mechlib
