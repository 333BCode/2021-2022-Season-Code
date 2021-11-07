#include "main.h"
#include "drivetrain.hpp"
#include "pros/misc.h"
#include "systems.hpp"
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
    using namespace motor_control;
	
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    constexpr double power = 2;
    bool usingExponential = false;

    bool intakeCommanded = false;

    bool usingManualControl = false;

    int count = 0;

    // sets break modes to coast
    base.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

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

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            
            usingManualControl = !usingManualControl;
            lift::setManualControl(usingManualControl);
            holder::setManualControl(usingManualControl);
            
            if (!usingManualControl) {
                lift::reset();
                holder::reset();
            }

        }

        if (usingManualControl) {

            if (count % 50 == 0) {
                controller.rumble(". . .");
                count = 0;
            }
            ++count;

            // shift key
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    holder::motor.move(50);
                } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    holder::motor.move(-50);
                } else {
                    holder::motor.move(0);
                }

            } else {

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                    lift::motor.move(50);
                } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    lift::motor.move(-50);
                } else {
                    lift::motor.move(0);
                }

            }

        } else {

            // shift key
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                    holder::toggleHolder();
                }

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                    stick::toggleStick();
                }

            } else {

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                    lift::toggleLift();
                }

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                    lift::toggleClamp();
                }

            }

        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake::reverse();
            intakeCommanded = false;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeCommanded = !intakeCommanded;
            if (intakeCommanded) {
                intake::intake();
            }
        } else if (!intakeCommanded) {
            intake::stop();
        }

        pros::Task::delay_until(&startTime, 10);

    }

}