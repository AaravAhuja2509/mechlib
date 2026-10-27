// A stand-in for the parts of PROS that mechlib uses, so the real mechlib code
// can run on a computer. Motor commands drive a simple physics model of a tank
// drivetrain (mass, turning inertia, motor torque and friction), and the fake
// encoders and IMU read back where the model says the robot is.
// Not part of the library: see sim/README.md.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#define PROS_ERR_F (INFINITY)

struct SimParams {
  double mass = 6.0;       // kg
  double J = 0.15;         // kg m^2
  double track = 0.292;    // m
  double r = 0.041275;     // wheel radius m
  double gear = 0.75;      // wheel turns per motor turn
  double wfree = 62.83;    // motor output rad/s at 12 V
  double Tu = 0.7;         // torque coefficient (N m at 12 V, before current limit)
  double Tlim = 0.35;      // current-limited torque (N m)
  double linStaticMv = 800, linKineticMv = 650, linViscous = 0.0;
  double rotStaticMv = 1330, rotKineticMv = 1100, rotViscous = 0.0;
  double sideBias = 1.0;   // physical right side strength
  int sensLatencyMs = 0;
  // Wiring: per configured motor, +1 = sign correct, -1 = sign wrong, 0 = unplugged.
  std::vector<int> leftDir{1, 1, 1}, rightDir{1, 1, 1};
  bool swapSides = false;  // configured left list is really the right side
  bool imuPresent = true;
  int imuSign = 1;         // -1 = mounted upside down
};

struct Sim {
  SimParams p;
  double v = 0, th = 0, w = 0, sL = 0, sR = 0;  // physical left/right side travel (m)
  int t = 0;
  double cmd[2] = {0, 0};      // per configured group, mV
  std::vector<double> perMotor[2];  // per-motor commands (NAN = use the group command)
  std::vector<double>& motorCmd(int g) { return perMotor[g]; }
  bool hold[2] = {false, false};
  struct Snap { double sL, sR, th; };
  std::deque<Snap> hist;
  double imuZero = 0;

  void reset() { v = th = w = sL = sR = 0; t = 0; cmd[0] = cmd[1] = 0; hold[0] = hold[1] = false; perMotor[0].clear(); perMotor[1].clear(); hist.clear(); imuZero = 0; }
  int physSide(int group) const { return p.swapSides ? 1 - group : group; }
  const std::vector<int>& dirs(int group) const { return group == 0 ? p.leftDir : p.rightDir; }

  double motorTorque(double mv, double motorW) {
    double T = p.Tu * (mv / 12000.0 - motorW / p.wfree);
    return std::fmax(-p.Tlim, std::fmin(p.Tlim, T));
  }
  double sideForceFromMv(double mv) { return 3 * p.Tu * (mv / 12000.0) / p.gear / p.r; }

  void step1ms() {
    const int N = 10;
    double dt = 0.001 / N;
    for (int k = 0; k < N; k++) {
      double sideV[2] = {v + w * p.track / 2, v - w * p.track / 2};
      double F[2] = {0, 0};
      for (int g = 0; g < 2; g++) {
        int ps = physSide(g);
        double mw = sideV[ps] / p.r / p.gear;
        const auto& ds = dirs(g);
        for (size_t i = 0; i < ds.size(); i++) {
          int d = ds[i];
          if (d == 0) continue;
          double c = (i < perMotor[g].size() && !std::isnan(perMotor[g][i])) ? perMotor[g][i] : cmd[g];
          double mv = hold[g] ? std::fmax(-12000, std::fmin(12000, -mw * 2000)) : d * c;
          F[ps] += motorTorque(mv, mw) / p.gear / p.r;
        }
      }
      F[1] *= p.sideBias;
      double Flin = F[0] + F[1];
      double FsLin = 2 * sideForceFromMv(p.linStaticMv), FkLin = 2 * sideForceFromMv(p.linKineticMv);
      if (std::fabs(v) < 1e-4 && std::fabs(Flin) <= FsLin) v = 0;
      else {
        double fr = (v != 0 ? (v > 0 ? 1 : -1) : (Flin > 0 ? 1 : -1)) * FkLin + p.linViscous * v;
        double nv = v + (Flin - fr) / p.mass * dt;
        if (v != 0 && (nv > 0) != (v > 0) && std::fabs(Flin) <= FsLin) nv = 0;
        v = nv;
      }
      double Trot = (F[0] - F[1]) * p.track / 2;
      double TsRot = 2 * sideForceFromMv(p.rotStaticMv) * p.track / 2, TkRot = 2 * sideForceFromMv(p.rotKineticMv) * p.track / 2;
      if (std::fabs(w) < 1e-3 && std::fabs(Trot) <= TsRot) w = 0;
      else {
        double fr = (w != 0 ? (w > 0 ? 1 : -1) : (Trot > 0 ? 1 : -1)) * TkRot + p.rotViscous * w;
        double nw = w + (Trot - fr) / p.J * dt;
        if (w != 0 && (nw > 0) != (w > 0) && std::fabs(Trot) <= TsRot) nw = 0;
        w = nw;
      }
      th += w * dt;
      sL += (v + w * p.track / 2) * dt;
      sR += (v - w * p.track / 2) * dt;
    }
    t++;
    hist.push_back({sL, sR, th});
    while ((int)hist.size() > p.sensLatencyMs + 1) hist.pop_front();
  }
  Snap sensed() { return hist.empty() ? Snap{sL, sR, th} : hist.front(); }
  double inches() const { return (sL + sR) / 2 / 0.0254; }
  double degrees() const { return th * 180 / M_PI; }
};
inline Sim& sim() { static Sim s; return s; }
inline std::function<void()>& stepHook() { static std::function<void()> f; return f; }

// Brain screen and controller capture, so tests can read what a driver would see.
struct Screens {
  std::string lcd[8];
  std::string ctrl[3];
  long version = 0;  // goes up every time the controller screen is written
  bool echoLcd = false;
};
inline Screens& screens() { static Screens s; return s; }

// Controller input: a queue of button presses, plus a policy called when it runs dry.
inline std::deque<int>& pressQueue() { static std::deque<int> q; return q; }
inline std::function<void()>& pressPolicy() { static std::function<void()> f; return f; }

// SD card: /usd/... is redirected to a folder on the PC.
inline std::string& usdDir() { static std::string d = "sim_usd"; return d; }
namespace std {
inline FILE* mock_fopen(const char* path, const char* mode) {
  std::string p = path;
  if (p.rfind("/usd/", 0) == 0) p = usdDir() + "/" + p.substr(5);
  return ::fopen(p.c_str(), mode);
}
}  // namespace std
#define fopen mock_fopen

namespace pros {
enum class MotorGearset { red, green, blue, invalid };
enum class MotorEncoderUnits { degrees };
enum class MotorBrake { coast, brake, hold };
enum controller_id_e_t { E_CONTROLLER_MASTER };
enum controller_analog_e_t { E_CONTROLLER_ANALOG_LEFT_X, E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X, E_CONTROLLER_ANALOG_RIGHT_Y };
enum controller_digital_e_t {
  E_CONTROLLER_DIGITAL_L1, E_CONTROLLER_DIGITAL_L2, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2,
  E_CONTROLLER_DIGITAL_UP, E_CONTROLLER_DIGITAL_DOWN, E_CONTROLLER_DIGITAL_LEFT, E_CONTROLLER_DIGITAL_RIGHT,
  E_CONTROLLER_DIGITAL_X, E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_Y, E_CONTROLLER_DIGITAL_A
};

inline void delay(int ms) {
  for (int i = 0; i < ms; i++) {
    sim().step1ms();
    if (stepHook()) stepHook()();
  }
}

inline int& groupCount() { static int c = 0; return c; }
inline std::vector<std::vector<std::int8_t>>& groupPorts() { static std::vector<std::vector<std::int8_t>> g; return g; }

class MotorGroup {
 public:
  int group;
  std::vector<std::int8_t> ports;
  std::vector<double> tare;
  MotorBrake mode = MotorBrake::coast;
  MotorGroup(const std::vector<std::int8_t>& p, MotorGearset) : group(groupCount()++ % 2), ports(p) {
    tare.assign(ports.size(), 0.0);
    groupPorts().push_back(p);
  }
  double motorDeg(size_t i) {
    const auto& d = sim().dirs(group);
    if (i >= d.size() || d[i] == 0) return PROS_ERR_F;
    auto s = sim().sensed();
    double m = sim().physSide(group) == 0 ? s.sL : s.sR;
    return d[i] * m / sim().p.r / sim().p.gear * 180.0 / M_PI;
  }
  std::vector<double> get_position_all() {
    std::vector<double> out;
    for (size_t i = 0; i < ports.size(); i++) out.push_back(motorDeg(i) - tare[i]);
    return out;
  }
  void tare_position_all() { for (size_t i = 0; i < ports.size(); i++) tare[i] = motorDeg(i); }
  void set_encoder_units_all(MotorEncoderUnits) {}
  void set_brake_mode_all(MotorBrake m) { mode = m; }
  void move_voltage(int mv) {
    mv = std::max(-12000, std::min(12000, mv));
    sim().cmd[group] = mv;
    sim().hold[group] = false;
    sim().perMotor[group].clear();
  }
  void move(int power) {
    power = std::max(-127, std::min(127, power));
    move_voltage(power * 12000 / 127);
  }
  void brake() {
    sim().cmd[group] = 0;
    sim().perMotor[group].clear();
    sim().hold[group] = mode != MotorBrake::coast;
  }
};

class Motor {
 public:
  std::int8_t port;
  explicit Motor(std::int8_t p) : port(p) {}
  // Find this port in the drive's groups.
  bool find(int& group, size_t& index) const {
    for (size_t g = 0; g < groupPorts().size(); g++) {
      const auto& ports = groupPorts()[g];
      for (size_t i = 0; i < ports.size(); i++) {
        if (ports[i] == port) { group = (int)g; index = i; return true; }
      }
    }
    return false;
  }
  double get_position() {
    int g; size_t i;
    if (!find(g, i)) return PROS_ERR_F;
    const auto& d = sim().dirs(g);
    if (i >= d.size() || d[i] == 0) return PROS_ERR_F;
    auto s = sim().sensed();
    double m = sim().physSide(g) == 0 ? s.sL : s.sR;
    return d[i] * m / sim().p.r / sim().p.gear * 180.0 / M_PI;
  }
  int move_voltage(int mv) {
    int g; size_t i;
    if (!find(g, i)) return 0;
    auto& per = sim().motorCmd(g);
    per.resize(sim().dirs(g).size(), NAN);
    per[i] = std::max(-12000, std::min(12000, mv));
    sim().hold[g] = false;
    return 1;
  }
};

class Imu {
 public:
  explicit Imu(std::uint8_t) {}
  double raw() const { return sim().p.imuSign * sim().sensed().th * 180.0 / M_PI; }
  double get_rotation() const { return sim().p.imuPresent ? raw() - sim().imuZero : PROS_ERR_F; }
  int reset(bool) { delay(sim().p.imuPresent ? 2000 : 0); sim().imuZero = raw(); return 1; }
  bool is_calibrating() const { return false; }
  int tare_rotation() { sim().imuZero = raw(); return 1; }
};

class Controller {
 public:
  explicit Controller(controller_id_e_t) {}
  int get_analog(controller_analog_e_t) { return 0; }
  bool get_digital_new_press(controller_digital_e_t button) {
    if (pressQueue().empty() && pressPolicy()) pressPolicy()();
    if (!pressQueue().empty() && pressQueue().front() == (int)button) {
      pressQueue().pop_front();
      return true;
    }
    return false;
  }
  template <typename... Args>
  int print(std::uint8_t line, std::uint8_t, const char* fmt, Args... args) {
    char buf[64];
    if constexpr (sizeof...(args) == 0) std::snprintf(buf, sizeof(buf), "%s", fmt);
    else std::snprintf(buf, sizeof(buf), fmt, args...);
    std::string s = buf;
    if (s.size() > 15) s = s.substr(0, 15);  // the real controller cuts at 15
    while (!s.empty() && s.back() == ' ') s.pop_back();
    if (line < 3) screens().ctrl[line] = s;
    screens().version++;
    return 1;
  }
  int rumble(const char*) { return 1; }
};

namespace lcd {
using lcd_btn_cb_fn_t = void (*)(void);
inline bool& initialized() { static bool b = false; return b; }
// Weak, like the real ones: mechlib checks they exist before using the screen.
// sim_main.cpp defines them.
__attribute__((weak)) bool is_initialized();
__attribute__((weak)) bool initialize();
inline lcd_btn_cb_fn_t& btn0() { static lcd_btn_cb_fn_t f = nullptr; return f; }
inline lcd_btn_cb_fn_t& btn2() { static lcd_btn_cb_fn_t f = nullptr; return f; }
inline void register_btn0_cb(lcd_btn_cb_fn_t cb) { btn0() = cb; }
inline void register_btn2_cb(lcd_btn_cb_fn_t cb) { btn2() = cb; }
template <typename... Args>
bool print(int line, const char* fmt, Args... args) {
  char buf[40];  // the real LLEMU keeps 39 characters per line
  if constexpr (sizeof...(args) == 0) std::snprintf(buf, sizeof(buf), "%s", fmt);
  else std::snprintf(buf, sizeof(buf), fmt, args...);
  if (line >= 0 && line < 8) {
    screens().lcd[line] = buf;
    if (screens().echoLcd && buf[0]) std::printf("    [brain %d] %s\n", line, buf);
  }
  return true;
}
}  // namespace lcd

namespace usd {
inline bool& installed() { static bool b = true; return b; }
inline bool is_installed() { return installed(); }
}  // namespace usd

namespace competition {
inline bool is_connected() { return false; }
}

class Task {
 public:
  template <class F>
  explicit Task(F) {}  // background tasks don't run in the simulator
};
}  // namespace pros

using pros::E_CONTROLLER_DIGITAL_A;
