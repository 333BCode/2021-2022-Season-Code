#include "autonomous.hpp"

void awp() {

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
    base.addAction(base.stopMotion, 6_in);
    base.addAction(base.stopMotion, 180_deg, true);
    base.moveTo(1.5_ft, 3_ft, 235);

    base << Waypoint {3_ft, 3_ft} << Waypoint {8_ft, 3_ft} >> Point {10_ft, 3_ft, 270_deg}.withAction(base.stopMotion, 0.5_in);

    holder.grab();

    base.limitSpeed(48);
    base << Waypoint {9.5_ft, 3_ft} << Waypoint {10_ft, 5.5_ft}.withAction(intake.intake, 12_ft);
    
    base.moveTo(10_ft, 1.5_ft);
    intake.stop();
    
}