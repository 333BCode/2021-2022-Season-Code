#include "autonomous.hpp"

void platformDownSide() {

    base.setPosition(0, 12_in, 90_deg);
    lift.release();

    base.moveTo(0, 57_in, goalRushExitConditions);
    lift.clamp();

    base.moveForward(-3.5_ft);

}