#include "drivetrain.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "systems.hpp"

/*
 * Drivetrain Devices
 */

// Easier reversing of the drivetrain (change 1 line instead of 6)
constexpr bool REVERSED_SET = false;

pros::Motor Drivetrain::frontLeftMotor          {1, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::topBackLeftMotor        {19, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};
pros::Motor Drivetrain::bottomBackLeftMotor     {19, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::frontRightMotor         {12, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};
pros::Motor Drivetrain::topBackRightMotor       {8, pros::E_MOTOR_GEARSET_06, REVERSED_SET};
pros::Motor Drivetrain::bottomBackRightMotor    {19, pros::E_MOTOR_GEARSET_06, !REVERSED_SET};

pros::Imu Drivetrain::inertial {15};

pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', true};
pros::ADIEncoder Drivetrain::middleEncoder  {'C', 'D', true};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

    pros::ADIDigitalOut Holder::clamp {'A'};

    pros::Motor Intake::motor {14, pros::E_MOTOR_GEARSET_18, true};

    pros::Motor         Lift::motor   {11, pros::E_MOTOR_GEARSET_36, true};
    pros::ADIDigitalOut Lift::claw    {'B'};

} // namespace motor_control