#include "systems/intake.hpp"

namespace motor_control::intake {

    constexpr int intakeSpeed = 77;

    void intake() {
        motor.move(intakeSpeed);
    }

    void reverse() {
        motor.move(-intakeSpeed);
    }

    void stop() {
        motor.move(0);
    }

} // namespace motor_control::intake