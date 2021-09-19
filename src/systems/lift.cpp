#include "systems/lift.hpp"
#include "pros/rtos.h"
#include "util/pid_controller.hpp"

namespace motor_control {

static PIDController liftPID {
    
    1,  // kP
    0,  // kD
    0,  // kI
    4,  // integralCap      (volts)
    
    0.5,  // timeToMaxVoltage (seconds)
    12, // maxVoltage       (volts)
    1,  // profilePower
    0   // startingVoltage  (volts)

};

constexpr long double highAngle = 35;
constexpr long double lowAngle = -35;

static bool liftIsUp            = false;
static bool usingManualControl  = false;

static bool clamping            = true;

static pros::Mutex liftMutex {};

namespace lift {

void raise() {
liftMutex.take(TIMEOUT_MAX);
    liftPID.setNewTarget(highAngle);
liftMutex.give();
    liftIsUp = true;
}

void lower() {
liftMutex.take(TIMEOUT_MAX);
    liftPID.setNewTarget(lowAngle);
liftMutex.give();
    liftIsUp = false;
}

void toggleLift() {
liftMutex.take(TIMEOUT_MAX);
    liftPID.setNewTarget(liftIsUp ? lowAngle : highAngle);
liftMutex.give();
    liftIsUp = !liftIsUp;
}

void setManualControl(bool manualControl) {
liftMutex.take(TIMEOUT_MAX);
    usingManualControl = manualControl;
    motor.move(0);
liftMutex.give();
}

void clamp() {
    claw.set_value(false);
    clamping = true;
}

void release() {
    claw.set_value(true);
    clamping = false;
}

void toggleClamp() {
    claw.set_value(clamping);
    clamping = !clamping;
}

} // namespace lift

void powerLift() {
liftMutex.take(TIMEOUT_MAX);
    if (!usingManualControl) {
        lift::motor.move_voltage(liftPID.calcPower(lift::motor.get_position()));
    }
liftMutex.give();
}

} // namespace motor_control