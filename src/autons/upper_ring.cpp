#include "autonomous.hpp"

void upperRing() {

    base.setPosition(targetTallNeutralMogo ? 22.5_in : 24_in, 1_ft, 0_deg);

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

    base.moveTo(2.5_ft, 57_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    base.moveForward(-3_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

}