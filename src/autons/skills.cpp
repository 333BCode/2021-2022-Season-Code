#include "autonomous.hpp"

void skills() {

   ExitConditions specialExitConditions {

      1.5,  // inches
      0.5,  // inches / second

      10, // degrees
      2,  // degrees per second

      0 // seconds

   };

   base.setPosition(108, 12, 90);

   // init
   holder::toggleHolder();
   lift::release();

   // goal rush
   base.moveTo(108_in, 5.0_ft, specialExitConditions);
   lift::clamp();

   base.setReversed(true);
   base.moveForward(-2.5_ft);
   lift::release();

   base.moveTo(108, 20, 215_deg);
   base.moveForward(-1.8_ft);
   holder::raise();
   intake::motor.move(85);

}