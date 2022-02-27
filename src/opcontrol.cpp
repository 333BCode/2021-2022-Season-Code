#include "main.h"
#include "drivetrain.hpp"
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
	
    /* Initialize controllers, state variables */

    // has all controls
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    // gets state data, can control intake, set Drivetrain brake mode to brake type hold (useful for climbing the platform)
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

    /* Set default states */

#ifndef DEFAULT_TO_MACROS_IN_OPCONTROL
    lift.setManualControl(true);
#endif

    base.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    bool usingBrakeTypeHold = false;

    bool holderOpen = true;
    secondaryController.clear();

    while (true) {

        uint32_t startTime = pros::millis();

        // get speed Drivetrain should move at
        int linearPow   = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rotPow      = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // deadzone for the sticks to prevent unwanted small movements
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
        // move the Drivetrain
        base.supply(linearPow, rotPow);
#endif

        // toggle if using macros or not
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            
            usingManualControl = !usingManualControl;
            lift.setManualControl(usingManualControl);
            
            liftIsUp = false;
            liftCommanded = true; // true so holdAngle properly updates

            if (!usingManualControl) {
                lift.reset(); // zero the lift
            }

            controller.rumble(".."); // signify change in control scheme

        }

        if (usingManualControl) { // if not using macros

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // raise lift
                lift.motor.move(127);
                liftCommanded = true;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // lower lift
                lift.motor.move(-127);
                liftCommanded = true;
            } else { // hold lift in place

                if (liftCommanded) { // tell the lift where to hold if it was just commanded
                    liftCommanded = false;
                    holdAngle = lift.motor.get_position();
                    if (holdAngle < 0) { // prevent the lift from trying to hold past the physical stop
                        holdAngle = 0;
                    }
                }

                // move the lift to the hold position
                // Better than using brake type hold as this prevents the lift from trying to
                // surpass the physical stop when it is all the way down
                lift.motor.move_absolute(holdAngle, 100);
            
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) { // clamp or release the claw
                lift.toggleClamp();
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { // zero the lift
                lift.reset();
            }

        } else { // if using macros

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { // raise or lower the lift
                liftIsUp = !liftIsUp;
                lift.toggleLift();
            }

            if (!liftIsUp) {

                // raise the lift high enough to intake rings (even when holding a mogo in the claw) if the lift is down
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
                    lift.toggleSubposition();
                }
                
            } else { // if the lift is up

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // raise the lift to max height
                    lift.setSubposition(Subposition::high);
                } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // lower the lift onto the platform
                    lift.setSubposition(Subposition::low);
                } else { // move the lift to the default raise height
                    lift.setSubposition(Subposition::neutral);
                }

            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // clamp or release the claw
                lift.toggleClamp();
            }
            
        }

        // grab or release with the holder, print holder state to the secondary controller
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            holder.toggle();
            holderOpen = !holderOpen;
            secondaryController.print(0, 0, holderOpen ? "open" : "closed");
        }

        if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            holder.grab();
        } else if (secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            holder.release();
        }

        // Toggle the Drivetrain between brake types hold (useful for climbing the platform) and coast,
        // update data displayed on the secondary controller
        if (secondaryController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            usingBrakeTypeHold = !usingBrakeTypeHold;
            if (usingBrakeTypeHold) {
                base.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
                secondaryController.print(1, 0, "holding");
            } else {
                base.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
                secondaryController.clear_line(1);
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)
            || secondaryController.get_digital(pros::E_CONTROLLER_DIGITAL_B)
        ) { // reverse the intake (when held, intake will stop when released)
            intake.reverse();
            intakeCommanded = false;
        } else if (
            (!liftIsUp
            && ((controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !usingManualControl)
            || (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && usingManualControl))
            ) || secondaryController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)
        ) { // toggle whether the intake spins forward or is stopped
            intakeCommanded = !intakeCommanded;
            if (intakeCommanded) {
                intake.intake();
                if (!liftIsUp) { // raise the lift enough to allow rings to pass under the claw (even when holding a mogo)
                    lift.setSubposition(Subposition::high);
                }
            } else if (!liftIsUp) { // lowers the lift if it was raised just high enough to let rings under the claw
                lift.setSubposition(Subposition::neutral);
            }
        } else if (!intakeCommanded) { // stop the intake if it was not toggled to spin forwards
            intake.stop();
        }

        pros::Task::delay_until(&startTime, 10);

    }

}