#ifndef _LIFT_HPP_
#define _LIFT_HPP_

#include "api.h"

/**
 * This file contains the declaration of the static Lift class
 *
 * This class provides functionality for the lift itself as well as the claw at the top of the lift
 *
 * This class has autonomous movement in mind, and can move the lift to several preset heights
 * A manual control mode is also available, which is useful for driver control of the lift
 * When manual control is enabled, access the public pros::Motor motor to control the lift
 * Do not access the pros::Motor motor when manual control is disabled, as a state machine in a
 * separate task will override commands,
 * and this shortcuts mutex protection used by the state machine when manual control is disabled
 */

namespace motor_control {

    class Lift final {
    public:

        // subpositions for the lift, in both the raised and lowered states
        enum class Subposition {
            // if the lift is raised, raise it to the max
            // if the lift is lowered, raises the lift enough to allow rings to pass under the claw and reach the intake
            high,
            // neutral position
            neutral,
            // if the lift is raised, lowers the lift onto the platform
            // if the lift is lowered, sets the lift to its neutral position
            low
        };

        // zero the lift integrated motor encoder and set it to use degrees for angular measurments
        // called in systemsTasks
        // MUTEX LOCKING
        static void init();

        // raises the lift, sets the subposition to neutral
        // MUTEX LOCKING
        static void raise();
        // lowers the lift, sets the subposition to neutral
        // MUTEX LOCKING
        static void lower();
        // lowers the lift if it is raised, otherwise raises it, sets the subposition to neutral
        // MUTEX LOCKING
        static void toggleLift();

        // returns true if the lift is currently raised, false otherwise
        // the current subposition is not considered
        // MUTEX LOCKING
        static bool isUp();

        // sets the current subposition of the lift to the one specified
        // MUTEX LOCKING
        static void setSubposition(Subposition subpos);
        // if the subposition is high, sets it to low, otherwise (even if subposition is neutral) sets the subposition to high
        // MUTEX LOCKING
        static void toggleSubposition();

        // sets the manual control status to the specified state
        // sets the lift to be lowered and in the neutral subposition (relevant if manual control is disabled)
        // MUTEX LOCKING
        static void setManualControl(bool manualControl);

        // closes the claw at the top of the lift
        static void clamp();
        // opens the claw at the top of the lift
        static void release();
        // if the claw is open, closes it, otherwise opens it
        static void toggleClamp();

        // returns true if the claw is currently closed, false otherwise
        static bool isClamping();

        // zeros the lift integrated motor encoder
        // MUTEX LOCKING
        static void reset();

        // This function runs the state machine that moves the lift to the appropriate position given the requested states
        // if manual control is disabled
        // This function uses the motor's built in PID controller to reach the desired position
        // Called in systems task
        // MUTEX LOCKING
        static void powerLift();

        // The motor for the physical lift
        // Defined in src/devices.cpp
        // Only access if manual control is enabled, otherwise the mutex guard
        // used by the state machine in systems task (through powerLift) is being bypassed
        // NEEDS MUTEX COVER, used in field control and systems tasks
        static pros::Motor motor;
        
    private:

        // pneumatic solinoid for the two claw cylinders
        // defined in src/devices.cpp
        static pros::ADIDigitalOut claw;

        // mutex to protect the pros::Motor motor when manual control is disabled
        static pros::Mutex mutex;

        // preset angles to target depending on lift states
        // heights are lowered, high enough for rings to pass under, lowered onto platform, above platform, and all the way up
        static const long double angles[5];

        // state variable to store whether the lift is currently up
        // NEEDS MUTEX COVER, used by field control and systems tasks
        static bool liftIsUp;
        // state variable to store the desired subposition of the lift
        // NEEDS MUTEX COVER, used by field control and systems tasks
        static Subposition subposition;
        // stores whether manual control is enabled
        // if true, the state machine will not power the lift
        // NEEDS MUTEX COVER, used by field control and systems tasks
        static bool usingManualControl;

        // stores whether the claw is currently closed or not, used by the toggle method
        static bool clamping;

    };

    // instance of Lift
    extern Lift lift;

} // namespace motor_control

#endif
