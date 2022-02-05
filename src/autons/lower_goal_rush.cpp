#include "autonomous.hpp"

void lowerGoalRush() {

    base.setPosition(9_ft, 12_in, 90_deg);
    lift.release();

    base.moveTo(8.8_ft, 56_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    if (!targetRings) {
        base.moveTo(9_ft, 1.8_ft, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);
    }

    base.moveTo(9_ft, 2.6_ft);
    Point pos = base.getPosition();
    if (sqrt((pos.x - 9_ft) * (pos.x - 9_ft) + (pos.y - 2.8_ft) * (pos.y - 2.8_ft)) > 4_in) {
        lift.release();
        base.supply(127, 0);
        pros::delay(200);
        base.moveTo(9_ft, 2.8_ft);
    }
    base.turnTo(180_deg);
    lift.release();

    base.moveTo(10.75_ft, 2.8_ft);

    holder.grab();

    pros::delay(500);

    lift.setSubposition(Subposition::high);

    base.moveTo(9.75_ft, 2.5_ft, 90_deg);
    intake.intake();

    base.limitLinearSpeed(20);
    base.moveForward(3_ft);

    base.unboundLinearSpeed();

    base.addAction(bundle(intake.stop(); holder.release(); lift.setSubposition(Subposition::neutral)), 1);
    base.moveTo(9.75_ft, 2.05_ft);

}