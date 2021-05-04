#include "main.h"
#include "drivetrain.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {

    using namespace drive;
	
    pros::Controller control(pros::E_CONTROLLER_MASTER);

    const double power = 2;
    bool usingExponential = false;

    while (true) {

        uint32_t startTime = pros::millis();

        int linearPow   = control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int strafePow   = control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int rotPow      = control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (abs(linearPow) < 10) {
            linearPow = 0;
        }
        if (abs(strafePow) < 10) {
            strafePow = 0;
        }
        if (abs(rotPow) < 10) {
            rotPow = 0;
        }

        if (usingExponential) {
            linearPow = (linearPow < 0 ? -1 : 1) * powf(abs(linearPow), power) / powf(127, power - 1);
            linearPow = (strafePow < 0 ? -1 : 1) * powf(abs(strafePow), power) / powf(127, power - 1);
            linearPow = (rotPow < 0 ? -1 : 1) * powf(abs(rotPow), power) / powf(127, power - 1);
        }

        base.supply(linearPow, strafePow, rotPow);

        if (control.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            usingExponential = !usingExponential;
        }

        if (control.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (base.getState() == State::enabledStrafing) {
                base(State::enabled);
            } else {
                base(State::enabledStrafing);
            }
        }

        pros::Task::delay_until(&startTime, 10);

    }

}