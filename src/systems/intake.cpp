#include "systems/intake.hpp"

constexpr int intakeSpeed = 127;

namespace motor_control::intake {

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