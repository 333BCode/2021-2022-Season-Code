#include "autonomous.hpp"

void platformUpSide() {

    lift::release();

    LinearExitConditions specialExitConditions = Drivetrain::defaultLinearExit<
        
        750,
        50,

        10000,
        200,

        150

    >;

    base.setPosition(108, 12, 90);

    // init
    holder::toggleHolder();

    // goal rush
    base.moveTo(108_in, 4.73_ft, specialExitConditions);
    // base.supply(30, 0);
    // pros::delay(500);
    lift::clamp();
    pros::delay(200);
    base.moveForward(-2.75_ft, false);
    lift::release();

    base.moveTo(108, 19, 215_deg);
    base.moveForward(-1.8_ft, false);
    holder::raise();
    pros::delay(800);
    intake::motor.move(85);

    pros::delay(3000);
    base.moveForward(18_in, false);
    intake::stop();

}