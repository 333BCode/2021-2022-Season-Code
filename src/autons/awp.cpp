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
    base.endEarly(6_in);
    base.addAction(base.stopMotion, 180_deg, true);
    base.moveTo(1.5_ft, 2.25_ft, 235_deg);

    base << Waypoint {3_ft, 2.5_ft} << Waypoint {8_ft, 3_ft};
    base.moveTo(10.75_ft, 3_ft, Drivetrain::defaultLinearExit<1000, 50000, 10000, 200000, 150, 200>);

    holder.grab();

    pros::delay(500);

    lift.setSubposition(Subposition::high);

    base.moveTo(9.9_ft, 2.65_ft, 90_deg);
    intake.intake();

    base.limitLinearSpeed(20);
    base.moveForward(3_ft);

    base.unboundLinearSpeed();

    base.addAction(bundle(intake.stop(); holder.release(); lift.setSubposition(Subposition::neutral)), 0.25);
    base.moveTo(10_ft, 2.25_ft);
    
}