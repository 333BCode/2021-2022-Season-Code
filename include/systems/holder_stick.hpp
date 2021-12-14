#ifndef _HOLDER_STICK_HPP_
#define _HOLDER_STICK_HPP_

#include "api.h"

namespace motor_control {

    namespace holder {

        void raise();
        void lower();
        void toggleHolder();

        void setManualControl(bool manualControl);

        void reset();

        extern pros::Motor motor;

    } // namespace holder

    namespace stick {

        void recieve();
        void deposit();
        void toggleStick();
        void setNeutral();

        extern pros::Motor motor;

    } // namespace stick

    void powerHolderAndStick();

} // namespace motor_control

#endif