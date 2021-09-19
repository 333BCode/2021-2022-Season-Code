#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"

namespace motor_control::intake {

    void intake();
    void reverse();
    void stop();

    extern pros::Motor motor;

} // namespace motor_control::intake

#endif