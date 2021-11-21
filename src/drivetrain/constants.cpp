#include "drivetrain.hpp"

motor_control::PIDController Drivetrain::linearPID {
    
    1.95,  // kP
    0.2,  // kD
    0,  // kI
    4,  // integralCap      (volts)
    
    0.5,  // timeToMaxVoltage (seconds)
    12, // maxVoltage       (volts)
    1,  // profilePower
    0   // startingVoltage  (volts)

};

motor_control::PIDController Drivetrain::rotPID {
    
    1.1,  // kP
    0.05,  // kD
    0,  // kI
    4,  // integralCap      (volts)
    
    0.5,  // timeToMaxVoltage (seconds)
    12, // maxVoltage       (volts)
    1,  // profilePower
    0   // startingVoltage  (volts)

};

const Drivetrain::ExitConditions Drivetrain::defaultExitConditions {

    1,  // inches
    0.5,  // inches / second

    10, // degrees
    2,  // degrees per second

    0.15 // seconds

};

const long double Drivetrain::defaultLookAheadDistance      = 30;
const long double Drivetrain::minDistForTurning             = 5;

const long double Drivetrain::wheelSpacingParallel          = 4.5;
const long double Drivetrain::wheelSpacingPerpendicular     = 0.2;
const long double Drivetrain::trackingWheelDiameter         = 2.81;

long double Drivetrain::maxVelocity             = 5;
long double Drivetrain::maxAcceleration         = 5;
const long double Drivetrain::drivetrainWidth   = 16;
long double Drivetrain::profileDT               = 0.01;