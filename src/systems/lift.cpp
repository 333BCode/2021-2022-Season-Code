#include "systems/lift.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"

namespace motor_control {

    // instance of Lift
    Lift lift {};

    /* initialize mutex, state variables */
    
    pros::Mutex Lift::mutex {};

    Lift::Subposition Lift::subposition {Lift::Subposition::neutral};

    bool Lift::liftIsUp            = false;
    bool Lift::usingManualControl  = false;

    bool Lift::clamping            = true;

    // zero the lift integrated motor encoder and set it to use degrees for angular measurments
    void Lift::init() {
    mutex.take();
        motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        motor.tare_position();
    mutex.give();
    }

    // sets the lift to raise, the subposition to neutral
    void Lift::raise() {
    mutex.take();
        liftIsUp = true;
        subposition = Subposition::neutral;
    mutex.give();
    }

    // sets the lift to lower, the subposition to neutral
    void Lift::lower() {
    mutex.take();
        liftIsUp = false;
        subposition = Subposition::neutral;
    mutex.give();
    }

    // sets the lift to lower if it is raised, otherwise sets it to raise, sets the subposition to neutral
    void Lift::toggleLift() {
    mutex.take();
        liftIsUp = !liftIsUp;
        subposition = Subposition::neutral;
    mutex.give();
    }

    // returns true if the lift is currently raised, false otherwise
    // the current subposition is not considered
    bool Lift::isUp() {
    mutex.take();
        bool liftState = liftIsUp;
    mutex.give();
        return liftState;
    }

    // sets the current subposition of the lift to the one specified
    void Lift::setSubposition(Subposition subpos) {
    mutex.take();
        subposition = subpos;
    mutex.give();
    }

    // if the subposition is high, sets it to low, otherwise (even if subposition is neutral) sets the subposition to high
    void Lift::toggleSubposition() {
    mutex.take();
        if (subposition == Subposition::high) {
            subposition = Subposition::low;
        } else {
            subposition = Subposition::high;
        }
    mutex.give();
    }

    // sets the manual control status to the specified state
    // sets the lift to be lowered and in the neutral subposition (relevant if manual control is disabled)
    void Lift::setManualControl(bool manualControl) {
    mutex.take();
        usingManualControl = manualControl;
        liftIsUp = false;
        subposition = Subposition::neutral;
    mutex.give();
    }

    // closes the claw
    void Lift::clamp() {
        claw.set_value(false);
        clamping = true;
    }

    // opens the claw
    void Lift::release() {
        claw.set_value(true);
        clamping = false;
    }

    // if the claw is open, closes it, otherwise opens it
    void Lift::toggleClamp() {
        claw.set_value(clamping);
        clamping = !clamping;
    }

    // returns true if the claw is currently closed, false otherwise
    bool Lift::isClamping() {
        return clamping;
    }

    // zeros the lift integrated motor encoder
    void Lift::reset() {
    mutex.take();
        motor.tare_position();
    mutex.give();
    }

    // This function runs the state machine that moves the lift to the appropriate position given the requested states
    // if manual control is disabled
    // This function uses the motor's built in PID controller to reach the desired position
    void Lift::powerLift() {
    mutex.take(20); // timeout and prevent deadlock if other task exits without freeing the mutex

        if (!usingManualControl) { // if other tasks are not controlling the lift

            size_t pos = (liftIsUp ? 3 : 0); // set index to high and neutral subposition if liftIsUp
            if (subposition == Subposition::high) { // go up an angle in angles if high subposition
                ++pos;
            } else if (subposition == Subposition::low && pos != 0) { // if raised and low subposition, go down an angle in angles
                --pos;
            }

            // move to the angle requested by the state machine
            motor.move_absolute(angles[pos], 100);

        }

    mutex.give();
    }

} // namespace motor_control