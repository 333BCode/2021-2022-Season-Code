#include "drivetrain.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "systems.hpp"

/*
 * Drivetrain Devices
 */
pros::Motor Drivetrain::frontLeftMotor  {1, pros::E_MOTOR_GEARSET_06, true};
pros::Motor Drivetrain::backLeftMotor   {19, pros::E_MOTOR_GEARSET_06, true};
pros::Motor Drivetrain::frontRightMotor {12, pros::E_MOTOR_GEARSET_06, false};
pros::Motor Drivetrain::backRightMotor  {8, pros::E_MOTOR_GEARSET_06, false};

pros::Imu Drivetrain::inertial {15};

pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', true};
pros::ADIEncoder Drivetrain::middleEncoder  {'C', 'D', true};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

    namespace holder {
        pros::Motor motor {9, pros::E_MOTOR_GEARSET_36, false};
    } // namespace holder

    namespace stick {
        pros::Motor motor {13, pros::E_MOTOR_GEARSET_36, false};
    } // namespace stick

    namespace intake {
        pros::Motor motor {14, pros::E_MOTOR_GEARSET_18, true};
    } // namespace intake

    namespace lift {
        pros::Motor motor           {11, pros::E_MOTOR_GEARSET_36, true};
        pros::ADIDigitalOut claw    {'B'};
    } // namespace lift

} // namespace motor_control