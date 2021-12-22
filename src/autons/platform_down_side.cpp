#include "autonomous.hpp"

void platformDownSide() {

#ifdef COMMENT

    lift::release();

    /*holder::toggleHolder();

    base.setPosition(24_in, 12_in, 90);
    base.moveTo(34_in, 4.7_ft, specialExitConditions);
    lift::clamp();
    base.setReversed(true);
    base.moveForward(-2.5_ft);*/

    base.setPosition(24, 12, 180);

    // init
    holder::toggleHolder();
    pros::delay(800);

    base.moveTo(40, 12, 180);

    holder::raise();
    pros::delay(800);
    intake::motor.move(85);
    base.moveTo(30, 12);

    pros::delay(3000);
    base.moveForward(6_in, false);
    intake::stop();
    //holder::lower();
/*
    // goal rush
    base.moveTo(72_in, 4.73_ft, specialExitConditions);
    // base.supply(30, 0);
    // pros::delay(500);
    lift::clamp();

    base.setReversed(true);
    base.moveForward(2.5_ft);
*/

#endif

}