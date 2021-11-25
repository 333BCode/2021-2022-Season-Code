#include "autonomous.hpp"

void skills() {

    lift::release();

    LinearExitConditions specialExitConditions = defaultLinearExit<
        
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

    base.setReversed(true);
    base.moveForward(-2.7_ft, false);
    base.turnTo(-45);
    lift::release();

    specialExitConditions = Drivetrain::defaultLinearExit<
        
        1000,
        50,

        10000,
        2000,

        50
    
    >;
    
    // base.moveTo(58, 60);
    base.moveTo(58.5, 59.5);
    stick::deposit();
    holder::raise();
    pros::delay(250);
    base.setReversed(false);
    base.moveForward(4.5_ft, false);

}