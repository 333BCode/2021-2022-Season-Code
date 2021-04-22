#include "drivetrain.hpp"

pros::Motor Drivetrain::frontLeftMotor  {12, false};
pros::Motor Drivetrain::backLeftMotor   {18, false};
pros::Motor Drivetrain::frontRightMotor {13, true};
pros::Motor Drivetrain::backRightMotor  {2, true};

pros::Imu Drivetrain::inertial {17};

pros::ADIEncoder Drivetrain::leftEncoder    {'A', 'B', true};
pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', false};
pros::ADIEncoder Drivetrain::middleEncoder  {'G', 'H', false};

Drivetrain::State state {Drivetrain::State::enabledStrafing};

const long double Drivetrain::wheelSpacingParallel          = 5.55;
const long double Drivetrain::wheelSpacingPerpendicular     = 4;
const long double Drivetrain::trackingWheelDiameter         = 2.75;