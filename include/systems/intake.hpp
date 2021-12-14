#ifndef _INTAKE_HPP_
#define _INTAKE_HPP_

#include "api.h"

namespace motor_control::intake {

    void intake();
    void reverse();
    void stop();

    extern pros::Motor motor;

} // namespace motor_control::intake

#endif