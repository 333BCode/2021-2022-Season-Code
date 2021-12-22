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
void systemsTasks(void*);
pros::Task mainTask(mainTasks);
pros::Task systemsTask(systemsTasks);

void mainTasks(void*) {

    DisplayControl displayControl {};

#if DISPLAY_DEBUG_LEVEL < 2
    bool displayActive = true;
#endif

    short frame = 0;

    Drivetrain::inertial.reset();
    while (Drivetrain::inertial.is_calibrating()) {
        pros::delay(10);
    }

    Drivetrain::rightEncoder.reset();
    Drivetrain::middleEncoder.reset();

Drivetrain::positionDataMutex.take(TIMEOUT_MAX);
    Drivetrain::calibrated = true;
Drivetrain::positionDataMutex.give();

    while (true) {

    Drivetrain::positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

#ifndef BRAIN_SCREEN_GAME_MODE
        Drivetrain::trackPosition();
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

        pros::Task::delay_until(&startTime, 10);

    }

}

void systemsTasks(void*) {
    motor_control::Lift::init();
    while (true) {
        uint32_t startTime = pros::millis();
        motor_control::Lift::powerLift();
        pros::Task::delay_until(&startTime, 10);
    }
}