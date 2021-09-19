#ifndef HOLDER_STICK_HPP
#define HOLDER_STICK_HPP

#include "api.h"

namespace motor_control {

    namespace holder {

        void raise();
        void lower();
        void toggleHolder();

        void setManualControl(bool manualControl);

        extern pros::Motor motor;

    } // namespace holder

    namespace stick {

        void recieve();
        void deposit();
        void toggleStick();
        void setNeutral();

        void setManualControl(bool manualControl);

        extern pros::Motor motor;

    } // namespace stick

    void powerHolderAndStick();

} // namespace motor_control

#endif