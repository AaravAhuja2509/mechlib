#include "mechlib/auton_selector.hpp"

#include <cstdio>
#include <utility>
#include <vector>

#include "api.h"
#include "screen.hpp"

namespace mechlib {
namespace auton {

namespace {

struct Routine {
  std::string name;
  std::function<void()> run;
};

std::vector<Routine> routines;
int selected = 0;

const char* SELECTION_FILE = "/usd/mechlib_auton.txt";
const int SCREEN_LINE = 7;

void show() {
  if (routines.empty()) {
    pros::lcd::print(SCREEN_LINE, "Auton: none added");
    return;
  }
  pros::lcd::print(SCREEN_LINE, "< Auton %d/%d: %s >", selected + 1, (int)routines.size(),
                   routines[selected].name.c_str());
}

void saveSelection() {
  if (!pros::usd::is_installed()) return;
  FILE* file = std::fopen(SELECTION_FILE, "w");
  if (!file) return;
  std::fprintf(file, "%d\n", selected);
  std::fclose(file);
}

void loadSelection() {
  if (!pros::usd::is_installed()) return;
  FILE* file = std::fopen(SELECTION_FILE, "r");
  if (!file) return;
  int saved = 0;
  if (std::fscanf(file, "%d", &saved) == 1 && saved >= 0 && saved < (int)routines.size()) {
    selected = saved;
  }
  std::fclose(file);
}

// The brain screen buttons call these.
void previous() {
  if (routines.empty()) return;
  selected = (selected + (int)routines.size() - 1) % (int)routines.size();
  show();
  saveSelection();
}

void next() {
  if (routines.empty()) return;
  selected = (selected + 1) % (int)routines.size();
  show();
  saveSelection();
}

}  // namespace

void add(const std::string& name, std::function<void()> routine) {
  routines.push_back({name, std::move(routine)});
}

void initialize() {
  loadSelection();
  if (!startScreen()) return;  // no brain screen in this project: use the saved choice
  pros::lcd::register_btn0_cb(previous);  // left brain button
  pros::lcd::register_btn2_cb(next);      // right brain button
  show();
}

void run() {
  if (routines.empty()) return;
  routines[selected].run();
}

}  // namespace auton
}  // namespace mechlib
