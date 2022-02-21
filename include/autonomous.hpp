#ifndef _AUTONOMOUS_HPP_
#define _AUTONOMOUS_HPP_

#include "main.h" // for forward declaration for autonomous
#include "drivetrain.hpp" // for drivetrain movement
#include "systems.hpp" // for subsystem 3 movement
#include "util/conversions.hpp" // for literals
#include "gui/display.hpp" // for DisplayControl::Auton, upperAutons, lowerAutons

/**
 * File to contain all includes needed for autonomous functions
 *
 * Additional macros are included to simplify auton writing
 *
 * Contains forward declarations for autonomous functions
 * auton points to the autonomous to be run
 */

using namespace drive;
using namespace motor_control;
using namespace conversions;

using Subposition = Lift::Subposition;

// make a series of function calls usable by Drivetrain as actions
#define bundle(funcs) []{funcs;}

// end a linear motion at a certain distance from the target
#define endEarly(dist) addAction(Drivetrain::stopMotion, dist)
// end a turn at a certain angle from the target
#define endTurnEarly(dist) addAction(Drivetrain::stopMotion, dist, true)

// end a pure pursuit movement with precision (finish with moveTo)
// call ex: base << Waypoint {x0, y0} << endAt(x1, y1);
#define endAt(x, y) Waypoint {x, y}; Drivetrain::moveTo(x, y)

using Auton     = DisplayControl::Auton;
using auton_t   = Auton::auton_t;

/**
 * auton and auton specification globals are accessed by the field control and lvgl tasks
 *
 * they are not protected by a mutex as the user should never (in general) be touching the brain screen as auton starts
 */

// function pointer to selected auton (updated in GUI, points to none by default)
extern auton_t auton;

// additional auton specifications to be used when relavent to autonomous functions
extern bool targetTallNeutralMogo;
extern bool targetShortNeutralMogo;
extern bool targetRings;

/*
 * Autonomous function forward declarations
 */

void none();
void upperGoalRush();
void upperRing();
void awp();
void lowerGoalRush();
void skills();

// specialized exit conditions for the goal rush
bool goalRushExitConditions(long double dist, bool firstLoop, bool reset);

#endif
