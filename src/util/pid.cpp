#include "util/pid.hpp"

namespace motor_control {

PID::PID(long double kP, long double kD, long double kI, long double integralCap,
    long double timeToMaxVoltage, long double maxVoltage, long double profilePower, long double startingVoltage)
    : kP {kP}, kD {kD}, kI {kI}, integralCap {integralCap},
    timeToMaxVoltage {timeToMaxVoltage},
    maxVoltage {(maxVoltage > 0 && maxVoltage <= 12 ? maxVoltage : 12)},
    profilePower {(profilePower >= 1 ? profilePower : 1)},
    startingVoltage {startingVoltage} {}

long double PID::getError() {
    return error;
}
long double PID::getDerivative() {
    return derivative;
}

void PID::setNewTarget(long double newTarget) {
    
    target      = newTarget;
    error       = 0;
    prevError   = 0;
    derivative  = 0;
    totalError  = 0;

    if (timeToMaxVoltage > 0) {

        timeElapsed     = 0;
        usingProfile    = true;

        /**
         * TODO:
         * Update profile start
         */
    }

}

void PID::alterTarget(long double newTarget) {
    target = newTarget;
}

int PID::calcPower(long double currPos) {



    return 0;
}

PID::AllConstants PID::getAllConstants() {
    return {
        kP, kD, kI, integralCap,
        timeToMaxVoltage, maxVoltage, profilePower, startingVoltage
    };
}

void PID::setConstants(long double newKP, long double newKD, long double newKI, long double newIntegralCap) {
    kP = newKP; kD = newKD; kI = newKI; integralCap = newIntegralCap;
}

void PID::setProfile(long double newTimeToMaxVoltage, long double newMaxVoltage,
    long double newProfilePower, long double newStartingVoltage)
{

    timeToMaxVoltage = newTimeToMaxVoltage;
    if (newTimeToMaxVoltage <= 0) {
        usingProfile = false;
    }

    maxVoltage = (newTimeToMaxVoltage > 0 && newTimeToMaxVoltage <= 12 ? newTimeToMaxVoltage : 12);
    profilePower = (newProfilePower >= 1 ? newProfilePower : 1);
    startingVoltage = newStartingVoltage;

}

} // namespace motor_control