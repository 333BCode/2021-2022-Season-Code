#include "drivetrain.hpp"

/*
 * Drivetrain Devices
 */
pros::Motor Drivetrain::frontLeftMotor  {12, false};
pros::Motor Drivetrain::backLeftMotor   {18, false};
pros::Motor Drivetrain::frontRightMotor {13, true};
pros::Motor Drivetrain::backRightMotor  {2, true};

pros::Imu Drivetrain::inertial {17};

pros::ADIEncoder Drivetrain::leftEncoder    {'A', 'B', true};
pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', false};
pros::ADIEncoder Drivetrain::middleEncoder  {'G', 'H', false};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

} // namespace motor_control