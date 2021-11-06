#include "drivetrain.hpp"
#include "systems.hpp"
#include "gui/display.hpp"
#include "macros.h"
#include "pros/rtos.h"

/**
 * Tasks
 */

void mainTasks(void*);
pros::Task sideTasks(mainTasks);

void mainTasks(void*) {

    DisplayControl displayControl {};

#if DISPLAY_DEBUG_LEVEL < 2
    bool displayActive = true;
#endif

    short frame = 0;

#ifdef USING_IMU
    Drivetrain::inertial.reset();
    {
        int count = 1; // in block so count gets removed
        while (Drivetrain::inertial.is_calibrating()) {
            pros::delay(10);
            if (count == 1) {
                ++count;
                std::cout << "IMU Reset not blocking.\n";
            }
        }
    }
#endif

    Drivetrain::leftEncoder.reset();
    // Drivetrain::rightEncoder.reset();
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

    motor_control::powerLift();
    motor_control::powerHolderAndStick();

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