#ifndef WINGS_HPP
#define WINGS_HPP

#include "api.h"

namespace motor_control::wings {

    enum class Side {
        left,
        right,
        both
    };

    void lower(Side wing = Side::both);
    void raise();

    extern pros::ADIDigitalOut leftWing;
    extern pros::ADIDigitalOut rightWing;

} // namespace motor_control::wings

#endif