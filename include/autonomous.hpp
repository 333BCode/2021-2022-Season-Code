#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "drivetrain.hpp"
#include "systems.hpp"
#include "pros/rtos.hpp"
#include "util/conversions.hpp"
#include "main.h"

using namespace drive;
using namespace motor_control;
using namespace conversions;

#define bundle(funcs) []{funcs;}
#define endAt(x, y) Waypoint {x, y}; Drivetrain::moveTo(x, y)

extern pros::Mutex autonSelectionMutex;

using auton_t = void(*)();

extern auton_t auton;

void skills();
void platformUpSide();
void platformDownSide();

#endif