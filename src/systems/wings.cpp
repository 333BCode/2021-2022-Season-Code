#include "systems/wings.hpp"

namespace motor_control::wings {

void lower(Side wing) {
    
    if (wing == Side::both) {

        leftWing.set_value(true);
        rightWing.set_value(true);

    } else if (wing == Side::left) {

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