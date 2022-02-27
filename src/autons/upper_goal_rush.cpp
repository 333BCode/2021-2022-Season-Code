#include "autonomous.hpp"

void upperGoalRush() {

    if (!targetRings) {

        base.addAction(lift.release, 12_ft);
        base.addAction(lift.clamp, 0.75_ft);
        base.endEarly(0.5_ft);
        base.moveForward(4.25_ft);
        base.moveForward(-4_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

    } else {

        base.addAction(lift.release, 12_ft);
        base.addAction(bundle(lift.clamp(); base.stopMotion()), 1_in);
        base.limitLinearSpeed(20);
        base.moveForward(67_in);
        base.unboundLinearSpeed();
        base.moveForward(-59_in, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

    }

/*
    base.setPosition(2_ft, 12_in, 90_deg);
    lift.release();

    base.moveTo(2.5_ft, 57_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    if (!targetRings) {
        base.moveForward(-3_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);
    }

    base.moveForward(-2_ft);
*/

}