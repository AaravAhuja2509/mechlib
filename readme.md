# mechlib

A beginner-friendly drivetrain library for VEX V5 robots running [PROS 4](https://pros.cs.purdue.edu/).

mechlib is made for rookie teams. You describe your robot in one block of code, and you get:

- **Driver control** in tank or arcade with one line.
- **`drive.move(24)`** to drive 24 inches straight, using PID and the IMU to stay on course.
- **`drive.turn(90)`** and **`drive.turnTo(180)`** for accurate turns.
- **A setup check** that tells you which motor ports are unplugged or need their sign flipped.
- **A guided PID tuner** on the controller. After every test it shows the overshoot, where the robot stopped and how long it took, plus a hint like `Too far: DN kP`.
- **Gains saved to the SD card**, loaded automatically when the robot starts.
- **An autonomous selector** on the brain screen.
- **Starting gains worked out for your robot** from its wheel size, gearing and track width, so it moves well before you tune anything.

```cpp
void autonomous() {
  drive.resetHeading();
  drive.move(24);    // forward 24 inches
  drive.turn(90);    // turn right 90 degrees
  drive.move(-12);   // back 12 inches
}
```

---

## Contents

1. [What you need](#what-you-need)
2. [Install](#install)
3. [Describe your robot](#describe-your-robot)
4. [First run](#first-run)
5. [Tune your PID](#tune-your-pid)
6. [Write your autonomous](#write-your-autonomous)
7. [Function reference](#function-reference)
8. [Troubleshooting](#troubleshooting)
9. [How it works](#how-it-works)
10. [Roadmap](#roadmap)
11. [Building mechlib yourself](#building-mechlib-yourself)

---

## What you need

- A PROS 4 project, kernel **4.2.0 or newer**, in VS Code with the PROS extension.
- A tank-style drivetrain: any number of V5 motors on each side.
- A V5 Inertial Sensor (IMU), mounted **flat, label side up**.
- liblvgl, for the brain screen. Every new PROS project already has it.
- Optional: a microSD card in the brain, to save your tuned gains.

## Install

1. Download `mechlib@1.0.0.zip` from the [Releases](../../releases) page and put it in your project folder.
2. In VS Code, open the PROS terminal (PROS sidebar, then **Integrated Terminal**) and run:

   ```
   pros c fetch mechlib@1.0.0.zip
   pros c apply mechlib
   ```

3. Open `include/main.h`, find the line that says `You can add C++-only headers here`, and add this under it:

   ```cpp
   #include "mechlib/mechlib.hpp"
   ```

4. Copy [`src/main.cpp`](src/main.cpp) from this repository into your project as a starting point. It has every feature wired up.

## Describe your robot

This block at the top of `main.cpp` is the only part you have to change:

```cpp
mechlib::Drive drive({
    .left_ports = {-1, -2, -3},
    .right_ports = {4, 5, 6},
    .imu_port = 10,
    .wheel_diameter = 3.25,
    .gear_ratio = 0.75,
    .cartridge = pros::MotorGearset::blue,
    .track_width = 12.0,
    .control = mechlib::ControlMode::ARCADE,
});
```

| Setting | What to put |
|---|---|
| `left_ports`, `right_ports` | The motor ports on each side. A **negative** port reverses that motor. Not sure which ones to reverse? Guess, then run the [setup check](#first-run). |
| `imu_port` | The port your Inertial Sensor is plugged into. |
| `wheel_diameter` | Wheel size in inches: 2.75, 3.25 or 4. |
| `gear_ratio` | Wheel turns for each motor turn. It's the motor gear's teeth divided by the wheel gear's teeth. A 36 tooth gear on the motor driving a 48 tooth gear on the wheel is 36 / 48 = **0.75**. Motors connected straight to the wheels are **1.0**. |
| `cartridge` | The motor cartridge: `blue` (600 RPM), `green` (200 RPM) or `red` (100 RPM). |
| `track_width` | Inches from the middle of the left wheels to the middle of the right wheels. |
| `control` | `ARCADE` (left stick drives forward and back, right stick turns) or `TANK` (each stick drives one side). |

Keep the settings in this order. You can leave out any of the last four and mechlib uses the defaults: gear ratio 1.0, blue cartridge, 12 inch track width and arcade control.

## First run

Upload the program and put the robot somewhere with a little room around it.

1. When the program starts, the brain screen says **calibrating IMU**. Keep the robot still for about 2 seconds.
2. Drive around. If anything drives the wrong way, run the setup check.
3. The example program gives you three practice buttons. They only work when the robot is **not** plugged into a field controller, so they can't go off during a match.

| Button | What it does |
|---|---|
| **Y** | **Setup check.** Each side spins for a moment, so the robot turns a little. The brain screen then lists anything wrong, like `Port 3: change -3 to 3` or `Port 5: no motor found`. |
| **X** | **PID tuner.** See [Tune your PID](#tune-your-pid). |
| **A** | Runs the autonomous routine picked on the brain screen. |

The brain screen layout:

| Line | Shows |
|---|---|
| 0 | Status messages |
| 1 to 2 | Distance driven and IMU rotation, live |
| 3 to 6 | Setup check results, or the tuner |
| 7 | The autonomous selector |

## Tune your PID

PID is how the robot decides how hard to push. **P** pushes harder the further away the target is. **D** brakes as the robot gets close, so it doesn't overshoot. **I** adds a little extra push when the robot is stuck just short of the target.

mechlib starts with gains worked out for your robot, so try it before you tune. When you want it better, press **X** to open the tuner.

### Tuner buttons

| Button | What it does |
|---|---|
| **A** | Run a test: drive 24 inches, or turn 180 degrees. Tests go forward then back, and right then left, so the robot stays in about the same place. |
| **UP / DOWN** | Raise or lower the value on screen by 10%. |
| **RIGHT / LEFT** | Next or previous step. |
| **X** | Put this value back to its default. |
| **Y** | Save all the gains to the SD card. They load automatically next time. |
| **B** | Close the tuner. |
| Sticks | Still drive, so you can line the robot up between tests. |

### Reading the controller

```
Drive kP  800.0     <- what you're tuning and its value
O0.4 E0.1 1.1s      <- overshoot, error when it stopped, time
Good! Next >        <- the hint
```

- **O** is how far it went past the target (inches, or degrees for turns).
- **E** is how far from the target it finally stopped.
- The last number is how long the move took, in seconds.

The brain screen shows the same numbers in full words.

### The steps

The tuner walks through 8 steps in order. For each one, press **A**, read the hint, change the value with **UP/DOWN**, and test again until the hint says **Good! Next >**. Then press **RIGHT**.

| Step | Goal |
|---|---|
| Drive kP | Gets the robot there quickly. Raise it if the robot is slow or stops short, lower it if it goes way past. |
| Drive kD | Stops the overshoot. Raise it until the robot stops going past, but not so much that it crawls at the end. |
| Drive kI | Finishes the last bit. Raise it if the robot stops short, lower it if it creeps past. |
| Turn kP, Turn kD, Turn kI | The same three steps, for turns. |
| Head kP | Raise it if the robot curves while driving straight. |
| Head kD | Raise it if the robot wiggles side to side while driving. |

The hints:

| Hint | What happened | What to do |
|---|---|---|
| `Too far: DN kP` | It went way past the target. | Press DOWN. |
| `Short: UP kP` | It stopped short of the target. | Press UP. |
| `Slow: UP kP` | It got there, but slowly. | Press UP. |
| `Overshot: UP kD` | It went past the target. | Press UP. |
| `Sluggish: DN kD` | It crawled at the end. | Press DOWN. |
| `Overshot: DN kI` | It crept past the target at the end. | Press DOWN. |
| `Short: UP kI` | It stopped short, or ran out of time. | Press UP. |
| `Good! Next >` | This step is done. | Press RIGHT. |

Going a little past the target is fine as long as the robot still stops close to it: within half an inch for drives and 1 degree for turns.

Turns are tested at 180 degrees because the bigger the turn, the faster the robot is spinning when it has to stop. Gains that stop a half turn cleanly also work for smaller turns.

When you're done, press **Y** to save.

### Where gains come from

1. mechlib starts with the defaults for your robot.
2. `drive.initialize()` replaces them with the gains saved on the SD card, if there are any.
3. Anything you set in code **after** `drive.initialize()` wins:

   ```cpp
   drive.driveGains = {800, 400, 60};   // kP, kI, kD
   drive.turnGains = {125, 313, 17.7};
   drive.headingGains = {156, 0, 8.3};
   ```

To go back to the defaults, delete `mechlib_gains.txt` from the SD card.

## Write your autonomous

```cpp
void leftSide() {
  drive.move(24);            // forward 24 inches
  drive.turn(-45);           // turn left 45 degrees
  drive.move(12, 1500, 50);  // 12 inches, give up after 1.5 seconds, 50% speed
  drive.turnTo(180);         // face the opposite way from the start
}

void initialize() {
  drive.initialize();
  mechlib::auton::add("Left side", leftSide);
  mechlib::auton::add("Right side", rightSide);
  mechlib::auton::initialize();
}

void autonomous() {
  drive.resetHeading();
  mechlib::auton::run();
}
```

Pick the routine with the left and right buttons under the brain screen. Your choice is saved on the SD card, so it survives a restart.

Every move and turn **remembers where the robot should be pointing**. If one turn stops a degree short, the next `move` drives on the heading you asked for, so small errors don't add up across a routine.

## Function reference

### `mechlib::Drive`

| Function | What it does |
|---|---|
| `Drive(config)` | Makes the drivetrain from your settings. |
| `initialize()` | Call in `initialize()`. Calibrates the IMU (about 2 seconds) and loads saved gains. |
| `driverControl(controller)` | Call every loop in `opcontrol()`. Drives with the sticks. |
| `move(inches, timeoutMs = 3000, maxSpeed = 100)` | Drives straight. Negative inches drive backward. `maxSpeed` is a percent. |
| `turn(degrees, timeoutMs = 2000, maxSpeed = 100)` | Turns by this many degrees. Positive turns right, negative turns left. |
| `turnTo(heading, timeoutMs = 2000, maxSpeed = 100)` | Turns to face a heading from 0 to 360, the short way. 0 is the way the robot faced at the last `resetHeading()`. |
| `resetHeading()` | The way the robot faces now becomes heading 0. Call at the start of `autonomous()`. |
| `checkSetup()` | The setup check. |
| `saveGains()`, `loadGains()` | Save or load the gains on the SD card. Return `false` if there's no card. |
| `defaultDriveGains()`, `defaultTurnGains()`, `defaultHeadingGains()` | The starting gains for your robot. |
| `getInches()` | Inches driven since the last move started. |
| `getRotation()` | IMU rotation in degrees. Keeps counting past 360. |
| `imuWorking()` | `true` if the IMU is plugged in and reading. |
| `topSpeed()` | Your robot's top speed in inches per second. |
| `driveGains`, `turnGains`, `headingGains` | The gains in use. You can change them in code. |

`move`, `turn` and `turnTo` return a `MoveResult` with `overshoot`, `finalError`, `headingDrift`, `timeMs` and `timedOut`, in case you want to print how a move went.

### `mechlib::auton`

| Function | What it does |
|---|---|
| `add(name, function)` | Adds a routine to the selector. Call in `initialize()`. |
| `initialize()` | Shows the selector. Call after your `add` calls. |
| `run()` | Runs the selected routine. Call in `autonomous()`. |

### `mechlib::runTuner(drive, controller)`

Opens the tuner. It takes over the controller until you press **B**.

## Troubleshooting

| Problem | Fix |
|---|---|
| The brain screen stays blank, or you see a warning about liblvgl | Your project is missing liblvgl. Run `pros c apply liblvgl` in the PROS terminal. |
| `pros c apply mechlib` says the template isn't compatible | Your kernel is older than 4.2. Run `pros c upgrade` in the PROS terminal. |
| `IMU NOT FOUND` on the brain screen | Check `imu_port` and the cable. Turns are skipped without the IMU, and drives finish on the motor encoders alone. |
| The robot spins and never finishes a turn, or curves harder and harder when driving | The robot turns the opposite way from what mechlib expects. Run the setup check (Y). Also make sure the IMU is mounted flat, label side up. |
| The setup check says every motor looks reversed | Three different mistakes look the same to the check, so drive with the sticks to tell them apart. Drives backward: flip the sign of every port. Turns the wrong way: swap the left and right lists. Drives fine: the IMU is upside down, so mount it label side up. |
| `No SD card!` in the tuner | Put a FAT32 microSD card in the brain. Gains still work without it, they just aren't saved. |
| Moves stop early or late on the field | Measure `wheel_diameter` and `gear_ratio` again. Every distance is worked out from them. |

## How it works

These details are for anyone who wants to learn what's happening inside, or change it.

- **The loop.** Moves run a loop every 10 ms. The PID output is in millivolts (12000 is full power), sent with `move_voltage`.
- **When a move is done.** The robot has to be within 0.5 inches of the target and moving slower than 2 inches per second, for 100 ms in a row. For turns it's 1 degree and 5 degrees per second. Friction can stop a robot just outside those limits, so a move also ends if the robot sits still within twice the limit (1 inch, or 2 degrees) for 250 ms. If neither happens, the move gives up at its timeout. `MoveResult.finalError` always says how far off it really was.
- **Slew.** The drive output can only grow by 600 mV per loop, so the robot doesn't wheelie or spin its wheels when it starts. Slowing down isn't limited.
- **Integral.** I only builds up close to the target (the integral zone), resets when the robot crosses the target, and is capped. That keeps it from causing big overshoot.
- **Derivative.** D uses the change in error per second. That's why kD is a small number next to kP.
- **Tuner hints.** Each test is compared to three limits: close enough (0.5 inches or 1 degree), way too far (1 inch or 2.5 degrees) and too slow (1.8 seconds for a half turn, and for drives a time worked out from the robot's top speed). The hints were checked in the [simulator](sim/README.md), where a pretend rookie follows every hint on 12 different robots, some starting from gains that are far too soft or far too strong.
- **Heading hold.** While driving, a second PID compares the IMU to the heading the robot should have, and steers by speeding up one side and slowing the other.
- **Default gains.** The defaults were tuned in a simulation of a reference robot (blue cartridge, 36:48 gearing, 3.25 inch wheels, 11.5 inch track width). Your robot gets the same gains scaled by how much faster or slower it drives and turns, so a slower robot gets bigger gains.
- **Setup check.** Each side is driven forward on its own. Driving the left side forward should turn the robot right, which the IMU sees. Each motor's encoder then shows whether that motor agreed with the way the side moved. If a side doesn't move at all, its motors are probably fighting each other, so the check flips one motor at a time until the side moves, which shows which one is backward.

The code is in [`src/mechlib`](src/mechlib), and the headers in [`include/mechlib`](include/mechlib) explain every function.

## Roadmap

- **v1.1:** Auto-tune. The robot runs its own tests and picks the gains.
- **v2:** Odometry (tracking the robot's position on the field), driving to points, and a tuning simulator that predicts good gains from a few logged runs.

Ideas and bug reports are welcome on the Issues page.

## Building mechlib yourself

This repository is a full PROS project. `src/main.cpp` is the example program, and everything in `src/mechlib` and `include/mechlib` is the library.

1. Open the folder in VS Code with the PROS extension.
2. **Build** and **Upload** as usual to try the example on a robot.
3. To make the installable zip, change `VERSION` in the `Makefile`, then run this in the PROS terminal:

   ```
   pros make template
   ```

   This makes `mechlib@<version>.zip`. Attach it to a new release on GitHub.

To check a change on 16 pretend robots before trying it on a real one, use the [simulator](sim/README.md).

The zip works on PROS kernels from the version it was built with up to the next major version.

## License

MIT. See [LICENSE](LICENSE). Made by VEX team 95187A.
