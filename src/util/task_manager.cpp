#include "drivetrain.hpp"
#include "gui/display.hpp"

/**
 * Tasks
 */

void mainTasks(void*);
pros::Task sideTasks(mainTasks);

void mainTasks(void*) {

    short frame = 0;

    Drivetrain::inertial.reset();
    do {
        pros::delay(10);
    } while (Drivetrain::inertial.is_calibrating());

    Drivetrain::leftEncoder.reset();
    Drivetrain::rightEncoder.reset();
    Drivetrain::middleEncoder.reset();

    while (true) {

        Drivetrain::positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        Drivetrain::trackPosition();
        
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