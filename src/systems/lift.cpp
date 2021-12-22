#include "systems/lift.hpp"
#include "pros/rtos.h"

namespace motor_control {

    Lift lift {};

    pros::Mutex Lift::mutex {};

    Lift::Subposition Lift::subposition {Lift::Subposition::neutral};

    const long double Lift::angles[5] = {0, 10, 45, 55, 65};

    bool Lift::liftIsUp            = false;
    bool Lift::usingManualControl  = false;

    bool Lift::clamping            = true;

    void Lift::init() {
    mutex.take();
        motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    mutex.give();
    }

    void Lift::raise() {
    mutex.take();
        liftIsUp = true;
        subposition = Subposition::neutral;
    mutex.give();
    }

    void Lift::lower() {
    mutex.take();
        liftIsUp = false;
        subposition = Subposition::neutral;
    mutex.give();
    }

    void Lift::toggleLift() {
    mutex.take();
        liftIsUp = !liftIsUp;
        subposition = Subposition::neutral;
    mutex.give();
    }

    bool Lift::isUp() {
    mutex.take();
        bool liftState = liftIsUp;
    mutex.give();
        return liftState;
    }

    void Lift::setSubposition(Subposition subpos) {
    mutex.take();
        subposition = subpos;
    mutex.give();
    }

    void Lift::setManualControl(bool manualControl) {
    mutex.take();
        usingManualControl = manualControl;
        liftIsUp = false;
    mutex.give();
    }

    void Lift::clamp() {
        claw.set_value(false);
        clamping = true;
    }

    void Lift::release() {
        claw.set_value(true);
        clamping = false;
    }

    void Lift::toggleClamp() {
        claw.set_value(clamping);
        clamping = !clamping;
    }

    bool Lift::isClamping() {
        return clamping;
    }

    void Lift::reset() {
    mutex.take();
        motor.tare_position();
    mutex.give();
    }

    void Lift::powerLift() {
    mutex.take();

        size_t pos = 0;
        if (liftIsUp) {
            switch (subposition) {
                case Subposition::neutral:
                    pos = 3;
                break;
                case Subposition::high:
                    pos = 4;
                break;
                default:
                    pos = 2;
            }
        } else if (subposition == Subposition::high) {
            pos = 1;
        }

        motor.move_absolute(angles[pos], 100);

    mutex.give();
    }

} // namespace motor_control