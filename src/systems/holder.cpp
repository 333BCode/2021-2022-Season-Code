#include "systems/holder.hpp"

namespace motor_control {

    // instance of Holder
    Holder holder {};

    // initialize state variable, used by Holder::toggle()
    bool Holder::clamping = false;

    // grab and hold onto a mogo in the holder
    void Holder::grab() {
        clamp.set_value(true);
        clamping = true;
    }

    // release a mogo held in the holder / open the holder to accept a mogo
    void Holder::release() {
        clamp.set_value(false);
        clamping = false;
    }

    // opens the holder if it is grabbing, otherwise closes it
    void Holder::toggle() {
        clamp.set_value(!clamping);
        clamping = !clamping;
    }

} // namespace motor_control