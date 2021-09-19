#include "drivetrain.hpp"
#include "pros/adi.hpp"
#include "systems.hpp"

/*
 * Drivetrain Devices
 */
pros::Motor Drivetrain::frontLeftMotor  {12, false};
pros::Motor Drivetrain::backLeftMotor   {18, false};
pros::Motor Drivetrain::frontRightMotor {13, true};
pros::Motor Drivetrain::backRightMotor  {2, true};

pros::Imu Drivetrain::inertial {17};

pros::ADIEncoder Drivetrain::leftEncoder    {'A', 'B', true};
// pros::ADIEncoder Drivetrain::rightEncoder   {'E', 'F', false};
pros::ADIEncoder Drivetrain::middleEncoder  {'G', 'H', false};

/*
 * Non Drivetrain Devices
 */
namespace motor_control {

    namespace holder {
        pros::Motor motor {3, false};
    } // namespace holder

    namespace stick {
        pros::Motor motor {4, false};
    } // namespace stick

    namespace intake {
        pros::Motor motor {2, false};
    } // namespace intake

    namespace lift {
        pros::Motor motor           {1, false};
        pros::ADIDigitalOut claw    {'E'};
    } // namespace lift

    namespace wings {
        pros::ADIDigitalOut leftWing    {'C'};
        pros::ADIDigitalOut rightWing   {'D'};
    } // namespace wings

} // namespace motor_control