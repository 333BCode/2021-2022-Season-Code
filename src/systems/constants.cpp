#include "systems.hpp"

/**
 * This file contains constants used in for subsystem 3 control
 */

namespace motor_control {

    // the speed Intake class methods will spin the motor at (on the interval [-127, 127])
    const int Intake::intakeSpeed = 127;

    // Angles of the lift motor from lowest to highest preset height
    const long double Lift::angles[5] = {
        
        0,  // fully lowered
        180, // slightly raised to intake rings
        395, // lower goal onto platform height
        650, // put goal onto platform height
        820  // fully raised
    
    };

} // namespace motor_control