#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "drivetrain.hpp"
#include "pros/rtos.hpp"
#include "util/conversions.hpp"
#include "main.h"

extern pros::Mutex autonSelectionMutex;

enum class Auton {
    skills,
    platformUpSide,
    platformDownSide
};

extern Auton auton;

void skills();
void platformUpSide();
void platformDownSide();

#endif