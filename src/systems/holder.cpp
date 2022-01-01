#include "systems/holder.hpp"

namespace motor_control {

    Holder holder {};

    bool Holder::clamping       = false;
    bool Holder::clampRequested = false;

    void Holder::grab() {
        clamp.set_value(true);
        clamping = true;
    }

    void Holder::release() {
        clamp.set_value(false);
        clamping = false;
    }

    void Holder::toggle() {
        clamp.set_value(!clamping);
        clamping = !clamping;
    }

    void Holder::requestGrabOnEnable() {
        clampRequested = true;
    }

    void Holder::grabIfRequested() {
        if (clampRequested) {
            clamp.set_value(true);
            clamping = true;
            clampRequested = false;
        }
    }

} // namespace motor_control