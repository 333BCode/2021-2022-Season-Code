#include "autonomous.hpp"

void skills() {

    /* lift::release();

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
    base.moveForward(4.5_ft, false); */

    // init

    lift::release();

    LinearExitConditions goalGrabExitConditions = Drivetrain::defaultLinearExit<
        
        1000,
        50,

        10000,
        2000,

        50

    >;

    base.setPosition(24, 12, 180);

    holder::toggleHolder();
    pros::delay(800);

    // get mogo off platform down side, score rings

    base.setReversed(true);
    base.moveTo(40, 12, 180);

    holder::raise();
    pros::delay(800);
    intake::motor.move(85);
    base.setReversed(false);
    base.moveTo(30, 12);

    pros::delay(3000);
    intake::stop();

    // get platform up side neutral mogo

    base.setReversed(true);

    base << Waypoint {35_in, 2.25_ft} << endAt(108_in, 2.25_ft);

    base.turnTo(90_deg);

    base.moveTo(108_in, 4.75_ft, goalGrabExitConditions);

    lift::clamp();

    // deposit opposing alliance mogo in opposing alliance zone

    base.moveTo(108_in, 9_ft, 180_deg, goalGrabExitConditions);

    base.setReversed(true);

    base.moveForward(-1.5_ft);
    holder::lower();
    pros::delay(500);

    // put neutral mogo on opposing alliance platform

    base.setReversed(false);

    base >> Point {6.5_ft, 9_ft, 135_deg}.withAction(bundle(lift::raise(); holder::raise()), 2_ft);

    base.moveForward(1_ft, false, goalGrabExitConditions);

    lift::release();

    // get platform up side alliance mogo

    base.setReversed(true);

    base >> Point {6.5_ft, 9_ft, 180_deg}.withAction(lift::lower, 20_deg, true);

    base.setReversed(false);

    base >> Point {36_in, 10_ft, 0_deg}.withAction(holder::lower, 1_ft);

    base.setReversed(true);

    base.moveForward(-2_ft);
    holder::raise();
    base.setReversed(false);
    pros::delay(500);

    // get platform down side neutral mogo

    base.moveTo(36_in, 10_ft, -90_deg);
    base.moveTo(36_in, 7.25_ft, goalGrabExitConditions);
    lift::clamp();

    // drop off alliance mogo
    base.moveTo(36_in, 3_ft, 0_deg, goalGrabExitConditions);
    holder::lower();
    pros::delay(500);

    // put neutral mogo on platform

    base >> Point {6.5_ft, 3_ft, -70_deg}.withAction(bundle(lift::raise(); holder::raise()), 2_ft);
    base.moveForward(1_ft, false, goalGrabExitConditions);

    lift::release();
    
    // grab dropped alliance mogo

    base >> Point {6.5_ft, 3_ft, 180_deg}.withAction(lift::lower, 20_deg, true);

    base.moveForward(3_ft, true, goalGrabExitConditions);

    lift::clamp();

    // score alliance mogo on platform

    base.setReversed(true);

    base >> Point {5.5_ft, 3_ft, -90_deg}.withAction(lift::raise, 2_ft);

    base.setReversed(false);

    base.moveForward(1_ft, false, goalGrabExitConditions);
    lift::release();

    base.setReversed(true);

    base.moveForward(-2_ft, false);

}