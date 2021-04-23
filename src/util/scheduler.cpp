#include "util/scheduler.hpp"
#include "drivetrain.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cstdint>

void mainTasks() {

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
        pros::Task::delay_until(&startTime, 10);

    }

}