#include "systems.hpp"

namespace motor_control {

    const int Intake::intakeSpeed = 127;

    const long double Lift::angles[5] = {
        
        0,  // fully lowered
        180, // slightly raised to intake rings
        395, // lower goal onto platform height
        550, // put goal onto platform height
        820  // fully raised
    
    };

} // namespace motor_control