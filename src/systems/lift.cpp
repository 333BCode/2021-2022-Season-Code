#include "systems/lift.hpp"
#include "pros/rtos.h"
#include "util/pid_controller.hpp"

namespace motor_control {

    constexpr long double highAngle = 2750;

    static bool liftIsUp            = false;
    static bool usingManualControl  = false;

    static bool clamping            = true;

    static pros::Mutex mutex {};

    namespace lift {

    void raise() {
    mutex.take(TIMEOUT_MAX);
        liftIsUp = true;
    mutex.give();
    }

    void lower() {
    mutex.take(TIMEOUT_MAX);
        liftIsUp = false;
    mutex.give();
    }

    void toggleLift() {
    mutex.take(TIMEOUT_MAX);
        liftIsUp = !liftIsUp;
    mutex.give();
    }

    void setManualControl(bool manualControl) {
    mutex.take(TIMEOUT_MAX);
        usingManualControl = manualControl;
        liftIsUp = false;
    mutex.give();
    }

    void clamp() {
        claw.set_value(false);
        clamping = true;
    }

    void release() {
        claw.set_value(true);
        clamping = false;
    }

    void toggleClamp() {
        claw.set_value(clamping);
        clamping = !clamping;
    }

    void reset() {
    mutex.take(TIMEOUT_MAX);
        motor.tare_position();
    mutex.give();
    }

    } // namespace lift

    void powerLift() {
    mutex.take(TIMEOUT_MAX);
        if (!usingManualControl) {
            lift::motor.move_absolute(liftIsUp ? highAngle : 0, 100);
        }
    mutex.give();
    }

} // namespace motor_control