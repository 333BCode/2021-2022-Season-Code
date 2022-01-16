#include "autonomous.hpp"

void awp() {

    base.setPosition(3_ft, 1_ft, 270_deg);

    base.setMaxSpeed(4000);
    base.moveForward(-10_in);

    holder.grab();
    base.addAction(intake.reverse, 8_in);
    base.addAction(intake.stop, 4_in);
    base.moveForward(16_in);

    pros::delay(500);

    intake.intake();
    pros::delay(2000);
    intake.stop();

    holder.release();
    base.moveForward(8_in);

    base.setMaxSpeed(12000);

    base << Waypoint {4_ft, 3_ft} << Waypoint {10_ft, 3_ft} >> Point {10_ft, 3_ft, 270_deg}.withAction(base.stopMotion, 0.5_in);
    base.setMaxSpeed(4000);
    base.moveForward(-8_in);

    holder.grab();

    base.setMaxSpeed(8000);
    base << Waypoint {9.5_ft, 3_ft} << Waypoint {10_ft, 5.5_ft}.withAction(intake.intake, 12_ft);
    
    base.moveTo(10_ft, 1.5_ft);
    intake.stop();
    
}