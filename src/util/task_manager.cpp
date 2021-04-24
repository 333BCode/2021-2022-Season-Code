#include "util/task_manager.hpp"
#include "drivetrain.hpp"
#include "gui/display.hpp"

void mainTasks() {

    static short frame = 0;

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
        Drivetrain::positionDataMutex.give();

        ++frame;
        if (frame >= 50) { // call updateOdomData every 0.5 seconds
            frame = 0;
            displayControl.updateOdomData();
        }

        pros::Task::delay_until(&startTime, 10);

    }

}