#include "drivetrain.hpp"
#include "gui/display.hpp"
#include "pros/rtos.h"

/**
 * Tasks
 */

void mainTasks(void*);
pros::Task sideTasks(mainTasks);

void mainTasks(void*) {

    short frame = 0;

#ifdef USING_IMU
    Drivetrain::inertial.reset();
    do {
        pros::delay(10);
    } while (Drivetrain::inertial.is_calibrating());
#endif

    Drivetrain::leftEncoder.reset();
    Drivetrain::rightEncoder.reset();
    Drivetrain::middleEncoder.reset();

    Drivetrain::calibrationMutex.take(TIMEOUT_MAX);
    Drivetrain::calibrated = true;
    Drivetrain::calibrationMutex.give();

    while (true) {

        Drivetrain::positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

#ifndef BRAIN_SCREEN_GAME_MODE
        Drivetrain::trackPosition();
#endif

        ++frame;
        if (frame >= 50) { // call updateOdomData every 0.5 seconds
            frame = 0;
            displayControl.updateOdomData(true);
        } else if (frame % 10 == 0) { // call every 0.1 seconds
            displayControl.updateOdomData(false);
        }

        Drivetrain::positionDataMutex.give();
        pros::Task::delay_until(&startTime, 10);

    }

}