#include "autonomous.hpp"

void platformUpSide() {

    lift::release();

    ExitConditions specialExitConditions {

      0.75,  // inches
      0.05,  // inches / second

      10, // degrees
      2,  // degrees per second

      0.15 // seconds

    };

    base.setPosition(108, 12, 90);

    // init
    holder::toggleHolder();

    // goal rush
    base.moveTo(108_in, 4.73_ft, specialExitConditions);
    // base.supply(30, 0);
    // pros::delay(500);
    lift::clamp();
    pros::delay(200);
    base.setReversed(true);
    base.moveForward(-2.75_ft);
    lift::release();

    base.moveTo(108, 19, 215_deg);
    base.moveForward(-1.8_ft);
    holder::raise();
    pros::delay(800);
    intake::motor.move(85);

    pros::delay(3000);
    base.setReversed(false);
    base.moveForward(18_in);
    intake::stop();

}