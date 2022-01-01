#ifndef _HOLDER_STICK_HPP_
#define _HOLDER_STICK_HPP_

#include "api.h"

namespace motor_control {

    class Holder final {
    public:

        static void grab();
        static void release();
        static void toggle();

        static void requestGrabOnEnable();
        static void grabIfRequested();

    private:

        static pros::ADIDigitalOut clamp;

        static bool clamping;
        static bool clampRequested;

    };

    extern Holder holder;

} // namespace motor_control

#endif