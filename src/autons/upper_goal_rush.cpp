#include "autonomous.hpp"

void upperGoalRush() {

    base.setPosition(2_ft, 12_in, 90_deg);
    lift.release();

    base.moveTo(3_ft, 57_in, goalRushExitConditions);
    lift.clamp();

    base.setLinearSlew(0);

    if (!targetRings) {
        base.moveForward(-2_ft, true, Drivetrain::defaultLinearExit<0, 0, 0, 0, 15000, 15000>);
    }

    base.moveForward(-2_ft);

}