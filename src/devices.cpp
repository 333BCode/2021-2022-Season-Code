#include "drivetrain.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "systems.hpp"

/*
 * Drivetrain Devices
 */

// Easier reversing of the drivetrain (change 1 line instead of 6)
constexpr bool REVERSED_SET = false;

pros::Motor Drivetrain::frontLeftMotor          {18, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::topBackLeftMotor        {20, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};
pros::Motor Drivetrain::bottomBackLeftMotor     {19, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::frontRightMotor         {6, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};
pros::Motor Drivetrain::topBackRightMotor       {9, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::bottomBackRightMotor    {7, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};

pros::Imu Drivetrain::imu1 {11};
pros::Imu Drivetrain::imu2 {12};

pros::Rotation      Drivetrain::parallelTrackingWheel       {1};
pros::ADIEncoder    Drivetrain::perpendicularTrackingWheel  {'A', 'B', true};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

    pros::ADIDigitalOut Holder::clamp {'D'};

    pros::Motor Intake::motor {8, pros::E_MOTOR_GEARSET_06, false};

    pros::Motor         Lift::motor   {10, pros::E_MOTOR_GEARSET_36, false};
    pros::ADIDigitalOut Lift::claw    {'C'};

} // namespace motor_control