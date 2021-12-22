#ifndef _INTAKE_HPP_
#define _INTAKE_HPP_

#include "api.h"

namespace motor_control {

    class Intake final {
    public:

        static void intake();
        static void reverse();
        static void stop();
        
        static pros::Motor motor;

        static const int intakeSpeed;

    };

    extern Intake intake;

} // namespace motor_control

#endif