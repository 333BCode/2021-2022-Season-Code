#include "systems/intake.hpp"

namespace motor_control {

    Intake intake {};

    const int Intake::intakeSpeed = 127;

    void Intake::intake() {
        motor.move(intakeSpeed);
    }

    void Intake::reverse() {
        motor.move(-intakeSpeed);
    }

    void Intake::stop() {
        motor.move(0);
    }

} // namespace motor_control