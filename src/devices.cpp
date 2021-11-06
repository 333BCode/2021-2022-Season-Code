#include "drivetrain.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "systems.hpp"

/*
 * Drivetrain Devices
 */
pros::Motor Drivetrain::frontLeftMotor  {12, pros::E_MOTOR_GEARSET_06, false};
pros::Motor Drivetrain::backLeftMotor   {18, pros::E_MOTOR_GEARSET_06, false};
pros::Motor Drivetrain::frontRightMotor {13, pros::E_MOTOR_GEARSET_06, true};
pros::Motor Drivetrain::backRightMotor  {2, pros::E_MOTOR_GEARSET_06, true};

pros::Imu Drivetrain::inertial {17};

pros::ADIEncoder Drivetrain::leftEncoder    {'A', 'B', true};
// pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', false};
pros::ADIEncoder Drivetrain::middleEncoder  {'G', 'H', false};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

    namespace holder {
        pros::Motor motor {3, pros::E_MOTOR_GEARSET_36, false};
    } // namespace holder

    namespace stick {
        pros::Motor motor {4, pros::E_MOTOR_GEARSET_36, false};
    } // namespace stick

    namespace intake {
        pros::Motor motor {2, pros::E_MOTOR_GEARSET_18, false};
    } // namespace intake

    namespace lift {
        pros::Motor motor           {1, pros::E_MOTOR_GEARSET_36, false};
        pros::ADIDigitalOut claw    {'E'};
    } // namespace lift

} // namespace motor_control