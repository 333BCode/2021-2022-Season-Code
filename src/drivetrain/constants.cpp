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

const long double Drivetrain::defaultLookAheadDistance      = 30;
const long double Drivetrain::minDistForTurning             = 5;

const long double Drivetrain::wheelSpacingParallel          = 2.7;
const long double Drivetrain::wheelSpacingPerpendicular     = 0.4;
const long double Drivetrain::trackingWheelDiameter         = 2.81;

long double Drivetrain::maxVelocity             = 50;
long double Drivetrain::maxAcceleration         = 50;
const long double Drivetrain::drivetrainWidth   = 16;
const long double Drivetrain::profileDT         = 0.01;