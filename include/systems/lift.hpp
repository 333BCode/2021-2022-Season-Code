#ifndef _LIFT_HPP_
#define _LIFT_HPP_

#include "api.h"

namespace motor_control {

    class Lift final {
    public:

        enum class Subposition {
            high,
            neutral,
            low
        };

        static void init();

        static void raise();
        static void lower();
        static void toggleLift();

        static bool isUp();

        static void setSubposition(Subposition subpos);

        static void toggleSubposition();

        static void setManualControl(bool manualControl);

        static void clamp();
        static void release();
        static void toggleClamp();

        static bool isClamping();

        static void reset();

        static void powerLift();

        static pros::Motor motor;
        
    private:

        static pros::ADIDigitalOut claw;

        static pros::Mutex mutex;

        static const long double angles[5];

        static bool liftIsUp;
        static Subposition subposition;
        static bool usingManualControl;

        static bool clamping;

    };

    extern Lift lift;

} // namespace motor_control

#endif