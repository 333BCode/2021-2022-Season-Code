#include "systems/lift.hpp"
#include "pros/rtos.h"
#include "util/pid_controller.hpp"

namespace motor_control {

    constexpr long double highAngle = 2750;

    static bool liftIsUp            = false;
    static bool usingManualControl  = false;

    static bool clamping            = true;
    static bool autoClamp           = true;

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
    mutex.take(TIMEOUT_MAX);
        claw.set_value(false);
        clamping = true;
    mutex.give();
    }

    void release() {
    mutex.take(TIMEOUT_MAX);
        claw.set_value(true);
        clamping = false;
    mutex.give();
    }

    void toggleClamp() {
    mutex.take(TIMEOUT_MAX);
        claw.set_value(clamping);
        clamping = !clamping;
    mutex.give();
    }

    void setAutoClamp(bool autoClampEnabled) {
    mutex.take(TIMEOUT_MAX);
        if (!clamping) {
            autoClamp = autoClampEnabled;
        }
    mutex.give();
    }

    bool isClamping() {
    mutex.take(TIMEOUT_MAX);
        bool clampState = clamping;
    mutex.give();
        return clampState;
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
        if (autoClamp) {
            if (clamping) {
                autoClamp = false;
            }
        }
    mutex.give();
    }

} // namespace motor_control