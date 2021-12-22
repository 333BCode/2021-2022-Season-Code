#ifndef _HOLDER_STICK_HPP_
#define _HOLDER_STICK_HPP_

#include "api.h"

namespace motor_control {

    class Holder final {
    public:

        static void grab();
        static void release();
        static void toggle();

    private:

        static pros::ADIDigitalOut clamp;

        static bool clamping;

    };

    extern Holder holder;

} // namespace motor_control

#endif