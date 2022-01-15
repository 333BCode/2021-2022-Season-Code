#ifndef _AUTONOMOUS_HPP_
#define _AUTONOMOUS_HPP_

#include "drivetrain.hpp" // for drivetrain movement
#include "systems.hpp" // for subsystem 3 movement
#include "util/conversions.hpp" // for literals
#include "main.h" // for forward declaration for autonomous

/**
 * File to contain all includes needed for autonomous functions
 *
 * Additional macros are included to simplify auton writing
 *
 * Contains auton_t alias, forward declarations for autonomous functions
 * auton points to the autonomous to be run
 * auton is protected by autonSelectionMutex
 */

using namespace drive;
using namespace motor_control;
using namespace conversions;

using Subposition = Lift::Subposition;

// make a series of function calls usable by Drivetrain as actions
#define bundle(funcs) []{funcs;}

// end a pure pursuit movement with precision (finish with moveTo)
// call ex: base << Waypoint {x0, y0} << endAt(x1, y1);
#define endAt(x, y) Waypoint {x, y}; Drivetrain::moveTo(x, y)

// protects auton function pointer
extern pros::Mutex autonSelectionMutex;

using auton_t = void(*)();

// function pointer to selected auton (updated in GUI, points to skills by default)
// NEEDS MUTEX COVER: used in field control and lvgl tasks
extern auton_t auton;

/*
 * Autonomous function forward declarations
 */

void skills();
void platformUpSide();
void platformDownSide();
void awp();

#endif