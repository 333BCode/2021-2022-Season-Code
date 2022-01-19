#include "autonomous.hpp"

void awp() {

    base.setPosition(3_ft, 1_ft, 180_deg);

    base.limitSpeed(24);
    base.moveForward(-10_in);

    holder.grab();
    base.addAction(intake.reverse, 8_in);
    base.addAction(intake.stop, 4_in);
    base.moveForward(16_in);

    intake.intake();
    pros::delay(1000);
    intake.stop();

    holder.release();
    base.addAction(base.stopMotion, 4_in);
    base.moveForward(12_in);

    base.unboundSpeed();

    base << Waypoint {4_ft, 3_ft} << Waypoint {10_ft, 3_ft} >> Point {10_ft, 3_ft, 270_deg}.withAction(base.stopMotion, 0.5_in);

    holder.grab();

    base.limitSpeed(48);
    base << Waypoint {9.5_ft, 3_ft} << Waypoint {10_ft, 5.5_ft}.withAction(intake.intake, 12_ft);
    
    base.moveTo(10_ft, 1.5_ft);
    intake.stop();
    
}