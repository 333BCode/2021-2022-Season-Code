#ifndef AUTONOMOUS_HPP
#define AUTONOMOUS_HPP

#include "drivetrain.hpp"
#include "pros/rtos.hpp"
#include "util/conversions.hpp"
#include "main.h"

extern pros::Mutex autonSelectionMutex;

using auton_t = void(*)();

extern auton_t auton;

void skills();
void platformUpSide();
void platformDownSide();

#endif