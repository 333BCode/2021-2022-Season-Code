#ifndef _INTAKE_HPP_
#define _INTAKE_HPP_

#include "api.h"

/**
 * This file contains the static Intake class
 *
 * Intake is essentially a wrapper for a single pros::Motor used for the physical intake
 *
 * This class is ment to be used solely in field control tasks,
 * as the intake never needs to spin to a specific position, just in a specific direction,
 * so we do not need to run control algorithms in a separate task
 */

namespace motor_control {

    class Intake final {
    public:

        // spins the intake forward (rings will be picked up and deposited on a mogo in the holder)
        static void intake();
        // spins the intake in reverse (rings will be spit out the front of the bot)
        static void reverse();
        // stops the intake from spinning
        static void stop();
        
        // the motor itself
        // this is public in case if we want to spin the intake at a different speed for whatever reason
        // declared in src/devices.cpp
        static pros::Motor motor;

        // the speed Intake class methods will spin the motor at (on the interval [-127, 127])
        // declared in src/systems/constants.cpp
        static const int intakeSpeed;

    };

    // instance of Intake
    extern Intake intake;

} // namespace motor_control

#endif