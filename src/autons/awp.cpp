#include "autonomous.hpp"

void awp() {

    base.setPosition(24_in, 1_ft, 0_deg);

    lift.setManualControl(true);

    lift.motor.move(75);
    do {
        pros::delay(10);
    } while (lift.motor.get_position() < 200);
    lift.release();
    pros::delay(200);

    lift.setManualControl(false);
    pros::delay(250);

    if (!targetShortNeutralMogo && !targetTallNeutralMogo) {

        base.endTurnEarly(10_deg);
        base.turnTo({2.75_ft, 5.75_ft});
        base.endEarly(0.5_ft);
        base.addAction(lift.clamp, 1_ft);
        base.moveTo(2.75_ft, 5.75_ft);

        base.endTurnEarly(10_deg);
        base.moveTo(2.25_ft, 2.9_ft, 180_deg);

        base.moveTo(9_ft, 3.1_ft, 180_deg);
        base.limitLinearSpeed(30);
        base.moveForward(-2_ft);
        holder.grab();
        pros::delay(500);

        intake.intake();
        base.moveForward(2_ft);
        pros::delay(1000);
        intake.stop();
        holder.release();

    } else {

        base.endEarly(6_in);
        base.addAction(base.stopMotion, 180_deg, true);
        base.moveTo(1.5_ft, 2.25_ft, 235_deg);

        base << Waypoint {3_ft, 2.5_ft} << endAt(9_ft, 3_ft);
        base.limitLinearSpeed(30);
        base.moveTo(11.75_ft, 3_ft); // , Drivetrain::defaultLinearExit<1000, 50000, 10000, 200000, 150, 200>);

        holder.grab();

        pros::delay(500);

        intake.intake();
        base.limitLinearSpeed(20);

        base.moveTo(9_ft, 3_ft);
        holder.release();

        base.unboundLinearSpeed();

        XYPoint endTarget;
        if (targetTallNeutralMogo) {

            endTarget = {6_ft, 6_ft};

        } else {

            endTarget = {9_ft, 6_ft};

        }

        base.turnTo(endTarget);

        intake.stop();

        base.endEarly(0.5_ft);
        base.addAction(lift.clamp, 1_ft);
        base.moveTo(endTarget.x, endTarget.y);

        base.moveTo(9_ft, 3_ft, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);

    }

}