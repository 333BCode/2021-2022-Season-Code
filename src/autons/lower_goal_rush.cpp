#include "autonomous.hpp"

void lowerGoalRush() {

    base.setPosition(9_ft, 12_in, 90_deg);
    lift.release();

    base.moveTo(9_ft, 57_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    base.moveTo(9_ft, 2.8_ft, 180_deg);
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

    base.addAction(bundle(intake.stop(); holder.release(); lift.setSubposition(Subposition::neutral)), 0.25);
    base.moveTo(9.75_ft, 2.05_ft);

}