#ifndef _HOLDER_STICK_HPP_
#define _HOLDER_STICK_HPP_

#include "api.h"

/**
 * This file contains the static Holder class
 *
 * Holder is essentially a wrapper for a single pros::ADIDigitalOut used for the physical holder
 *
 * This class is ment to be used solely in field control tasks;
 * due to the binary state nature of pneumatics we do not need control algorithms for the holder to run in a separate task
 */

namespace motor_control {

    class Holder final {
    public:

        // grab and hold onto a mogo in the holder
        static void grab();
        // release a mogo held in the holder / open the holder to accept a mogo
        static void release();
        // opens the holder if it is grabbing, otherwise closes it
        static void toggle();

    private:

        // the pneumatic cylinder, defined in src/devices.cpp
        static pros::ADIDigitalOut clamp;

        // state variable to track the current state, used by the toggle method
        static bool clamping;

    };

    // instance of Holder
    extern Holder holder;

} // namespace motor_control

#endif