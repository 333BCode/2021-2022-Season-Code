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
    using Subposition = Lift::Subposition;
	
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::Controller secondaryController(pros::E_CONTROLLER_PARTNER);

    bool intakeCommanded = false;

#ifdef DEFAULT_TO_MACROS_IN_OPCONTROL
    bool liftIsUp           = lift.isUp();
    bool usingManualControl = false;
    bool liftCommanded      = true;
#else
    bool liftIsUp           = false;
    bool usingManualControl = true;
    bool liftCommanded      = true;
#endif
    double holdAngle        = 0;

#ifndef DEFAULT_TO_MACROS_IN_OPCONTROL
    lift.setManualControl(true);
#endif

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

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            
            usingManualControl = !usingManualControl;
            lift.setManualControl(usingManualControl);
            
            liftIsUp = false;
            liftCommanded = true;

            if (!usingManualControl) {
                lift.reset();
            }

            controller.rumble("..");

        }

        if (usingManualControl) {

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                lift.motor.move(127);
                liftCommanded = true;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                lift.motor.move(-127);
                liftCommanded = true;
            } else {

                if (liftCommanded) {
                    liftCommanded = false;
                    holdAngle = lift.motor.get_position();
                }

                if (holdAngle < 0) {
                    holdAngle = 0;
                }
                lift.motor.move_absolute(holdAngle, 100);
            
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
                lift.toggleClamp();
            }

        } else {

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                liftIsUp = !liftIsUp;
                lift.toggleLift();
            }

            if (!liftIsUp) {

                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                    lift.toggleSubposition();
                }
                
            } else {

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    lift.setSubposition(Subposition::high);
                } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                    lift.setSubposition(Subposition::low);
                } else {
                    lift.setSubposition(Subposition::neutral);
                }

            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                lift.toggleClamp();
            }
            
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            holder.toggle();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)
            || secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_B)
        ) {
            intake.reverse();
            intakeCommanded = false;
        } else if (
            (!liftIsUp
            && ((controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !usingManualControl)
            || (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && usingManualControl))
            ) || secondaryController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)
            || secondaryController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)
        ) {
            intakeCommanded = !intakeCommanded;
            if (intakeCommanded) {
                intake.intake();
                if (!liftIsUp) {
                    lift.setSubposition(Subposition::high);
                }
            } else if (!liftIsUp) {
                lift.setSubposition(Subposition::neutral);
            }
        } else if (!intakeCommanded) {
            intake.stop();
        }

        pros::Task::delay_until(&startTime, 10);

    }

}