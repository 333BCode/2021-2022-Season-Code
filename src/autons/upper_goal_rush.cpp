#include "autonomous.hpp"

void upperGoalRush() {

    base.setPosition(2_ft, 1.5_ft, 70_deg);

    if (!targetRings) {

        base.addAction(lift.release, 12_ft);
        base.addAction(lift.clamp, 0.75_ft);
        base.endEarly(0.5_ft);
        base.moveForward(4.2_ft);
        lift.lower();
        base.moveForward(-4.25_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

    } else {

        base.addAction(lift.release, 12_ft);
        base.addAction(lift.clamp, 0.75_ft);
        base.endEarly(0.5_ft);
        base.moveForward(4.2_ft);
        lift.lower();
        base.addAction(lift.release, 1_ft);
        base.moveTo(1.5_ft, 1.3_ft, [](long double curDist, bool, bool){
            return (curDist < 4_in);
        });
        lift.release();
        base.moveTo(1.5_ft, 1.3_ft);
        base.turnTo(160_deg);
        base.addAction(holder.grab, 4_in);
        base.limitLinearSpeed(20);
        base.moveTo(3.35_ft, 1.1_ft);
        holder.grab();
        base.unboundLinearSpeed();
        pros::delay(500);
        base.endEarly(2_in);
        base.moveForward(12_in, false);
        intake.intake();
        pros::delay(5000);
        intake.stop();
        holder.release();

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