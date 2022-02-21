#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "systems.hpp"
#include "gui/display.hpp"
#include "macros.h"
#include "pros/rtos.h"

/**
 * This file contains user created tasks
 *
 * mainTasks concerns everything relating to the Drivetrain
 * systemsTasks concerns everything relating to subsystem 3
 */

/* Initialize tasks */

void mainTasks(void*);
pros::Task mainTask(mainTasks);

void systemsTasks(void*);
pros::Task systemsTask(systemsTasks);

// mainTasks calibrates / resets Drivetrain sensors, marks calibration as complete, and then runs odometry and updates the GUI
void mainTasks(void*) {

    // create / initialize the GUI on the brain screen
    DisplayControl displayControl {};

#ifndef DISPLAY_DEBUG
    bool displayActive = true;
#endif

    // keep track of cycles so the GUI does not have to be updated every 10 milliseconds
    short frame = 0;

    // calibrate / reset Drivetrain sensors
    Drivetrain::imu1.reset();
    Drivetrain::imu2.reset();
    while (Drivetrain::imu1.is_calibrating() || Drivetrain::imu2.is_calibrating()) {
        pros::delay(10);
    }

    Drivetrain::parallelTrackingWheel.reset();
    Drivetrain::perpendicularTrackingWheel.reset();

Drivetrain::positionDataMutex.take(20); // timeout and prevent deadlock if other task exits without freeing the mutex
    Drivetrain::calibrated = true; // mark calibration as complete
Drivetrain::positionDataMutex.give();

    while (true) {

    Drivetrain::positionDataMutex.take(20);

        uint32_t startTime = pros::millis();

#ifndef BRAIN_SCREEN_GAME_MODE
        Drivetrain::trackPosition(); // run odometry and update tracked position
#endif

    Drivetrain::positionDataMutex.give();

#ifndef DISPLAY_DEBUG
        if (displayActive) {
            if (pros::competition::is_disabled()) {
#endif
                ++frame;
                if (frame >= 50) { // update odom data text every 0.5 seconds, move the virtual bot
                    frame = 0;
                    displayControl.updateOdomData(true);
                } else if (frame % 10 == 0) { // move the virtual bot every 0.1 seconds
                    displayControl.updateOdomData(false);
                }
#ifndef DISPLAY_DEBUG
            } else {
                displayActive = false;
                displayControl.cleanScreen();
            }
        }
#endif

        // critical to run this loop as often as possible as allowed by the sensors (100Hz)
        pros::Task::delay_until(&startTime, 10);

    }

}

// zeros the lift and sets it to use degrees, then runs a state machine to supply power to the lift
void systemsTasks(void*) {
    motor_control::Lift::init();
    while (true) {
        uint32_t startTime = pros::millis();
        motor_control::Lift::powerLift();
        pros::Task::delay_until(&startTime, 10);
    }
}