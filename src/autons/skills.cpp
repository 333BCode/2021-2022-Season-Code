#include "autonomous.hpp"

void skills() {

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
   base.moveForward(-2.7_ft);
   base.turnTo(-45);
   lift::release();

   specialExitConditions = ExitConditions {

      1,  // inches
      0.05,  // inches / second

      10, // degrees
      2,  // degrees per second

      0.05 // seconds

   };
   // base.moveTo(58, 60);
   base.moveTo(58.5, 59.5);
   stick::deposit();
   holder::raise();
   pros::delay(250);
   base.setReversed(false);
   base.moveForward(4.5_ft);

}