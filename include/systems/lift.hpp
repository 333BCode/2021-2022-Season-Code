#ifndef LIFT_HPP
#define LIFT_HPP

#include "api.h"

namespace motor_control {

namespace lift {

    void raise();
    void lower();
    void toggleLift();

    void setManualControl(bool manualControl);

    void clamp();
    void release();
    void toggleClamp();

    void setAutoClamp(bool autoClampEnabled);
    bool isClamping();

    void reset();

    extern pros::Motor motor;
    extern pros::ADIDigitalOut claw;

} // namespace lift

void powerLift();

} // namespace motor_control

#endif