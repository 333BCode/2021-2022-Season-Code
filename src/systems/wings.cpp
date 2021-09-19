#include "systems/wings.hpp"

namespace motor_control::wings {

void lower(side::side wing) {
    
    if (wing == side::BOTH) {

        leftWing.set_value(true);
        rightWing.set_value(true);

    } else if (wing == side::LEFT) {

        leftWing.set_value(true);

    } else {

        rightWing.set_value(true);
        
    }

}

void raise() {

    leftWing.set_value(false);
    rightWing.set_value(false);

}


} // namespace motor_control::wings