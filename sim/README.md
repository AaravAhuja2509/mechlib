# mechlib simulator

This folder is a test bench. It runs the real mechlib code on your computer, with a pretend robot instead of a real one. You don't need it to use mechlib. It's for checking that a change to the library didn't break anything before you try it on the robot.

- `mock/api.h` stands in for PROS. The motors, IMU, controller, brain screen and SD card are all fake. Motor voltages drive a simple physics model of a tank drive: its weight, how hard it is to spin, motor torque and friction.
- `sim_main.cpp` runs three test suites against that pretend robot.

## Run it

You need a C++ compiler for your computer. That's a different one from the compiler PROS uses for the brain.

- **Windows:** install WSL (Ubuntu), then run `sudo apt install g++` in it.
- **Mac:** run `xcode-select --install`.
- **Linux:** install `g++`.

Then, from the main mechlib folder:

```
g++ -std=gnu++20 -O1 -I sim/mock -I include src/mechlib/*.cpp sim/sim_main.cpp -o mechsim
./mechsim motion
./mechsim setup
./mechsim tuner
```

`./mechsim` on its own runs all three. Gains the tuner saves go in a `sim_usd` folder, the pretend SD card.

| Suite | What it does | What good looks like |
|---|---|---|
| `motion` | Drives 16 different robots through 8 moves and turns each, using the default gains. The robots cover different gearings, wheel sizes, 4, 6 and 8 motors, a heavy robot, high and low friction, sensor lag and a weak side. | Nothing times out, and every move ends within about 1 inch and 1 degree. |
| `setup` | Runs the setup check on 10 wiring mistakes. | Each mistake gets the right message on the brain screen. |
| `tuner` | A pretend rookie does exactly what every tuner hint says on 12 robots, some starting from bad gains. It gives up on a step after 15 tests. Then the tuned gains are tried on other move and turn sizes. | It never gives up, and nothing times out after tuning. |

The last line of each suite is a summary. These are the results for v1.0:

```
MOTION: 128 moves and turns, 0 timed out. Worst: 0.78 in off on a move, 1.06 deg off on a turn.
TUNER: 12 robots, 118 tests, gave up on 0 steps. After tuning: 0 of 72 moves and turns timed out. Worst: 0.80 in off on a move, 0.95 deg off on a turn.
```

To see every test the pretend rookie runs, and the hint it got:

```
ROOKIE_VERBOSE=1 ./mechsim tuner
```

## What it can't tell you

It's a model, not a robot. It has no wheel slip, battery sag, motor heating or bumps on the field, and its friction numbers are estimates. Use it to compare one version of the code with another, then check the result on the real robot.
