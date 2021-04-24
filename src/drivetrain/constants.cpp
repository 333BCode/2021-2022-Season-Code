#include "drivetrain.hpp"

Drivetrain::State Drivetrain::state {Drivetrain::State::enabledStrafing};

const long double Drivetrain::wheelSpacingParallel          = 5.55;
const long double Drivetrain::wheelSpacingPerpendicular     = 4;
const long double Drivetrain::trackingWheelDiameter         = 2.75;