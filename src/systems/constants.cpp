#include "systems.hpp"

namespace motor_control {

    const int Intake::intakeSpeed = 127;

    const long double Lift::angles[5] = {
        
        0,  // fully lowered
        10, // slightly raised to intake rings
        45, // lower goal onto platform height
        55, // put goal onto platform height
        65  // fully raised
    
    };

} // namespace motor_control