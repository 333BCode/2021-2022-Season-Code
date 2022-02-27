# 2021 - 2022 Main Robot Code

Most of the code is dedicated to accurate and efficient drivetrain movements. Certain algorithms are used to achieve this.

PID - This is a control algorithm that takes a target state and the current state and suggests how much power to supply to the motors.

Odometry - This tracks our position on the field. The current position can be used as state data for the PID controllers powering the drivetrain.

Pure Pursuit - This is a path following algorithm for arcing between waypoints. Our PID controllers can target points determined by pure pursuit for smoother motions.

Motion Profiling - This is a primarily feedforward path generating and following algorithm, with pure pursuit used for feedback error correction.

PID tells our motors to spin, odometry is used as an input to the drivetrain PID controllers, pure pursuit is used to tell the drivetrain where to move before reaching a final end goal, and motion profiling is a separate feedforward algorithm that both generates a path and supplies power to the drivetrain.

## Creating Autons

Subsystem 3 controls are found in the header files in include/systems. Drivetrain controls are found in include/drivetrain.hpp and the header files in include/drivetrain.

### Custom Exit Conditions

If the logic of the default exit conditions works, just pass different template arguments. Otherwise, create a new function with the function signature of the respective exit condition function pointer (found in include/drivetrain.hpp), and possibly have Drivetrain friend that function.

## Updating Devices

Update src/devices.cpp.

## Adding a New Auton

Add autonomous functions to files in src/autons, and forward declare them in include/autonomous.hpp. In src/autonomous.cpp, add a DisplayControl::Auton (declared in include/gui/display.hpp), with autonFunc pointing to the auton function, to either the upperAutons or lowerAutons arrays. Update the respective array length in include/gui/display.hpp.

## Macros

In include/macros.h: #define BRAIN_SCREEN_GAME_MODE to drive the virtual bot on the brain screen, #define DISPLAY_DEBUG to prevent deletion of most of the gui once the bot is enabled by field control.

## Locations of Code Components

Drivetrain Movement Methods to Call -> include/drivetrain.hpp, include/drivetrain/path.hpp, include/drivetrain/point.hpp

PID -> src/util/pid_controller.cpp, include/util/pid_controller.hpp

Odometry -> src/drivetrain/drivetrain.cpp

Pure Pursuit, Movement Algorithms -> src/drivetrain/movement.cpp

Motion Profiling -> src/drivetrain/path.cpp, include/drivetrain/path.hpp, src/util/equations.cpp, include/util/equations.hpp

GUI -> scr/gui, include/gui

Tasks -> src/util/task_manager.cpp

Literals -> include/util/conversions.hpp
