#include "autonomous.hpp"

// default auton to one that does nothing
void none() {}
auton_t auton = none;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

/* Initialize array to determine GUI auton button creation */

const Auton DisplayControl::upperAutons[] {{upperGoalRush, "rush", true}, {upperRing, "ring", true}, {awp, "awp", true}};
const Auton DisplayControl::lowerAutons[] {{lowerGoalRush, "rush", true}, {skills, "skills"}};

/* Initialize auton specifiers */

bool targetTallNeutralMogo  {false};
bool targetShortNeutralMogo {false};
bool targetRings            {false};

void autonomous() {

    // set Drivetrain to standard states
    Drivetrain::waitUntilCalibrated();
    Drivetrain::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    Drivetrain::setFollowDirection(Direction::autoDetermine);

    lift.setManualControl(false); // allow state machine to power the lift

    auton(); // run the selected autonomous function

}

// specialized exit conditions for the goal rush
bool goalRushExitConditions(long double dist, bool firstLoop, bool reset) {

    static int count = 0;
    Point pos = base.getPosition();
    ++count;
    if (count > 250) { // stop if 2.5 seconds have elapsed
        return true;
    }
    return pos.y >= 57 || dist < 0.5; // stop if close enough to target or far enough forward

}