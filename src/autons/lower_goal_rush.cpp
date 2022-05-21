#include "autonomous.hpp"

void lowerGoalRush() {

    base.setPosition(9_ft, 12_in, 90_deg);
    
    base.addAction(lift.release, 12_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.endEarly(0.225_ft);
    if (targetTallNeutralMogo) {
        base.moveTo(6.15_ft, 5.45_ft);
    } else {
        base.moveForward(4.00_ft);
    }

    if (!targetRings) {
        base.moveTo(9_ft, 1.8_ft, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);
    }

    base.moveTo(9_ft, 2.5_ft);
    Point pos = base.getPosition();
    /*if (sqrt((pos.x - 9_ft) * (pos.x - 9_ft) + (pos.y - 2.8_ft) * (pos.y - 2.8_ft)) > 4_in) {
        lift.release();
        base.supply(127, 0);
        pros::delay(200);
        base.moveTo(9_ft, 2.8_ft);
    }*/
    base.turnTo(180_deg);
    if (targetTallNeutralMogo) {
        pros::delay(500);
    }
    lift.release();

    base.endEarly(1_in);
    base.moveTo(10.5_ft, 2.75_ft);

    holder.grab();

    pros::delay(500);

    lift.setSubposition(Subposition::high);

    base.moveTo(9.75_ft, 2.5_ft, 90_deg);
    intake.intake();

    base.limitLinearSpeed(20);
    base.moveForward(3_ft);

    base.unboundLinearSpeed();

    base.addAction(bundle(intake.stop(); lift.setSubposition(Subposition::neutral)), 1);
    base.moveTo(9.75_ft, 2.05_ft);
    holder.release();

}