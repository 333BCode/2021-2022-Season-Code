#include "systems/intake.hpp"

namespace motor_control {

    // instance of Intake
    Intake intake {};

    // spins the intake forward (rings will be picked up and deposited on a mogo in the holder)
    void Intake::intake() {
        motor.move(intakeSpeed);
    }

    // spins the intake in reverse (rings will be spit out the front of the bot)
    void Intake::reverse() {
        motor.move(-intakeSpeed);
    }

    // stops the intake from spinning
    void Intake::stop() {
        motor.move(0);
    }

} // namespace motor_control