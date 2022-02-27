#include "drivetrain.hpp"

/**
 * This file contains constants used throughout the Drivetrain class and the algorithms it implements
 */

motor_control::PIDController Drivetrain::linearPID {
    
    1.75,  // kP
    0.08,  // kD
    0,  // kI
    4,  // integralCap          (volts)
    
    0,  // voltageAcceleration (volts / seconds) ------------ was 16
    12, // maxVoltage           (volts)
    0   // startingVoltage      (volts)

};

motor_control::PIDController Drivetrain::rotPID {
    
    1.1,  // kP
    0.05,  // kD
    0,  // kI
    4,  // integralCap          (volts)
    
    24,  // voltageAcceleration (volts / seconds)
    12, // maxVoltage           (volts)
    0   // startingVoltage      (volts)

};

// in inches, pure pursuit and motion profiling error correction constant
const long double Drivetrain::defaultLookAheadDistance      = 30;
// moveTo constant, prevents turning when very close to target
// (so the bot does not turn if very slightly to the side of the target)
const long double Drivetrain::minDistForTurning             = 5;

/* odometry constants, all in inches */
const long double Drivetrain::wheelSpacingParallel          = 1.75;
const long double Drivetrain::wheelSpacingPerpendicular     = -0.1;
const long double Drivetrain::trackingWheelDiameter         = 2.81;

/* motion profiling constants */
const long double Drivetrain::maxVelocity             = 60; // in / s
const long double Drivetrain::maxAcceleration         = 40; // in / s^2
const long double Drivetrain::drivetrainWidth   = 6.125; // in
const long double Drivetrain::profileDT         = 0.01; // s