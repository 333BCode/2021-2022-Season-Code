#include "main.h"
#include "drivetrain.hpp"
#include "pros/rtos.h"
#include "macros.h"

#ifdef BRAIN_SCREEN_GAME_MODE
#include "util/conversions.hpp"
#endif

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

    constexpr double power = 2;
    bool usingExponential = false;

    // sets break modes to coast
    base.stop();

    while (true) {

        uint32_t startTime = pros::millis();

        int linearPow   = control.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rotPow      = control.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (abs(linearPow) < 10) {
            linearPow = 0;
        }
        if (abs(rotPow) < 10) {
            rotPow = 0;
        }

        if (usingExponential) {
            linearPow = (linearPow < 0 ? -1 : 1) * powf(abs(linearPow), power) / powf(127, power - 1);
            rotPow = (rotPow < 0 ? -1 : 1) * powf(abs(rotPow), power) / powf(127, power - 1);
        }

#ifdef BRAIN_SCREEN_GAME_MODE
        base.positionDataMutex.take(TIMEOUT_MAX);
            base.setPosition(
                base.xPos + linearPow * cos(conversions::radians(base.heading)) / 317.5,
                base.yPos + linearPow * sin(conversions::radians(base.heading)) / 317.5,
                base.heading - rotPow / 70.55
            );
        base.positionDataMutex.give();
#else
        base.supply(linearPow, rotPow);
#endif

        if (control.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            usingExponential = !usingExponential;
        }

        pros::Task::delay_until(&startTime, 10);

    }

}