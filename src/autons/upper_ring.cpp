#include "autonomous.hpp"

void upperRing() {

    base.setPosition(2_ft, 1_ft, 0_deg);

    lift.setManualControl(true);

    lift.motor.move(75);
    do {
        pros::delay(10);
    } while (lift.motor.get_position() < 200);
    lift.release();
    pros::delay(200);

    lift.setManualControl(false);
    pros::delay(250);

    base.endEarly(0.5_ft);
    base.moveForward(-1.5_ft);

    base.moveTo(targetTallNeutralMogo ? 6_ft : 3_ft, 57_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    base.moveForward(-2_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

}