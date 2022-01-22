#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "systems.hpp"
#include "gui/display.hpp"
#include "macros.h"
#include "pros/rtos.h"

/**
 * Tasks
 */

void mainTasks(void*);
pros::Task mainTask(mainTasks);
#ifdef DEFAULT_TO_MACROS_IN_OPCONTROL
void systemsTasks(void*);
pros::Task systemsTask(systemsTasks);
#endif

void mainTasks(void*) {

    DisplayControl displayControl {};

#if DISPLAY_DEBUG_LEVEL < 2
    bool displayActive = true;
#endif

    short frame = 0;

    /*Drivetrain::imu1.reset();
    Drivetrain::imu2.reset();
    while (Drivetrain::imu1.is_calibrating() || Drivetrain::imu2.is_calibrating()) {
        pros::delay(10);
    }*/

    Drivetrain::parallelTrackingWheel.reset_position();
    Drivetrain::perpendicularTrackingWheel.reset();

Drivetrain::positionDataMutex.take(20);
    Drivetrain::calibrated = true;
Drivetrain::positionDataMutex.give();

#ifndef DEFAULT_TO_MACROS_IN_OPCONTROL
    motor_control::Lift::init();
#endif

    while (true) {

    Drivetrain::positionDataMutex.take(20);

        uint32_t startTime = pros::millis();

#ifndef BRAIN_SCREEN_GAME_MODE
        // Drivetrain::trackPosition();
#endif

    Drivetrain::positionDataMutex.give();

#ifndef DISPLAY_DEBUG
        if (displayActive) {
            if (pros::competition::is_disabled()) {
#endif
                ++frame;
                if (frame >= 50) { // call updateOdomData every 0.5 seconds
                    frame = 0;
                    displayControl.updateOdomData(true);
                } else if (frame % 10 == 0) { // call every 0.1 seconds
                    displayControl.updateOdomData(false);
                }
#ifndef DISPLAY_DEBUG
            } else {
                displayActive = false;
                displayControl.cleanScreen();
            }
        }
#endif
#ifndef DEFAULT_TO_MACROS_IN_OPCONTROL
        motor_control::Lift::powerLift();
#endif

        pros::Task::delay_until(&startTime, 10);

    }

}

#ifdef DEFAULT_TO_MACROS_IN_OPCONTROL
void systemsTasks(void*) {
    motor_control::Lift::init();
    while (true) {
        uint32_t startTime = pros::millis();
        motor_control::Lift::powerLift();
        pros::Task::delay_until(&startTime, 10);
    }
}
#endif