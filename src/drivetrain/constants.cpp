#include "drivetrain.hpp"

motor_control::PIDController Drivetrain::linearPID {
    
    1,  // kP
    0,  // kD
    0,  // kI
    4,  // integralCap      (volts)
    
    2,  // timeToMaxVoltage (seconds)
    12, // maxVoltage       (volts)
    3,  // profilePower
    0   // startingVoltage  (volts)

};

motor_control::PIDController Drivetrain::rotPID {
    
    1,  // kP
    0,  // kD
    0,  // kI
    4,  // integralCap      (volts)
    
    2,  // timeToMaxVoltage (seconds)
    12, // maxVoltage       (volts)
    3,  // profilePower
    0   // startingVoltage  (volts)

};

const Drivetrain::ExitConditions Drivetrain::defaultExitConditions {

    4,  // inches
    1,  // inches / second

    10, // degrees
    5,  // degrees per second

    0.5 // seconds

};

const long double Drivetrain::defaultLookAheadDistance      = 5;
const long double Drivetrain::minDistForTurning             = 6;

const long double Drivetrain::wheelSpacingParallel          = 5.55;
const long double Drivetrain::wheelSpacingPerpendicular     = 4;
const long double Drivetrain::trackingWheelDiameter         = 2.75;