#include "autonomous.hpp"

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

const Auton DisplayControl::upperAutons[] {{skills, "skills"}, {awp, "awp", true}, {platformDownSide, "base"}};
const Auton DisplayControl::lowerAutons[] {{platformUpSide, "base"}};

bool targetTallNeutralMogo  {false};
bool targetShortNeutralMogo {false};
bool targetRings            {false};

void autonomous() {

    Drivetrain::waitUntilCalibrated();
    Drivetrain::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    Drivetrain::setFollowDirection(Direction::autoDetermine);

#ifndef DEFAULT_TO_MACROS_IN_OPCONTROL
    lift.setManualControl(false); // if macro defined, make sure to not enable manual control before auton (should not happen)
#endif

    auton();

}

bool goalRushExitConditions(long double dist, bool firstLoop, bool reset) {

    static int count = 0;
    Point pos = base.getPosition();
    ++count;
    if (count > 250) {
        return true;
    }
    return pos.y >= 55 || dist < 0.5;

}