#ifndef WINGS_HPP
#define WINGS_HPP

#include "api.h"

namespace motor_control::wings {

    namespace side {
        enum side {
            LEFT,
            RIGHT,
            BOTH
        };
    }

    void lower(side::side wing = side::BOTH);
    void raise();

    extern pros::ADIDigitalOut leftWing;
    extern pros::ADIDigitalOut rightWing;

} // namespace motor_control::wings

#endif