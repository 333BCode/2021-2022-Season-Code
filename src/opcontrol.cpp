#include "main.h"
#include "drivetrain.hpp"
#include "pros/misc.h"
#include "systems.hpp"
#include "pros/rtos.h"
#include "macros.h"
#include "systems/intake.hpp"
#include "systems/lift.hpp"

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
    using namespace motor_control;
	
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    constexpr double power = 2;
    bool usingExponential = false;

    bool intakeCommanded = false;

    // sets break modes to coast
    base.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    wings::raise();

    while (true) {

        uint32_t startTime = pros::millis();

        int linearPow   = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rotPow      = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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
        Point position = base.getPosition();
        base.setPosition(
            position.x + linearPow * cos(conversions::radians(position.heading)) / 317.5,
            position.y + linearPow * sin(conversions::radians(position.heading)) / 317.5,
            position.heading - rotPow / 70.55
        );
#else
        base.supply(linearPow, rotPow);
#endif

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            usingExponential = !usingExponential;
        }

        // shift key
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                lift::toggleLift();
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                lift::toggleClamp();
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                intake::intake();
                intakeCommanded = true;
            }

        } else {

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                holder::toggleHolder();
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                stick::toggleStick();
            }

        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake::reverse();
        } else if (!intakeCommanded) {
            intake::stop();
        }
        intakeCommanded = false;

        pros::Task::delay_until(&startTime, 10);

    }

}