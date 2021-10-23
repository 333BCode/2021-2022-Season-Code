#include "util/pid_controller.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace motor_control {

PIDController::PIDController(long double kP, long double kD, long double kI, long double integralCap,
    long double timeToMaxVoltage, long double maxVoltage, long double profilePower, long double startingVoltage)
    : kP {kP}, kD {kD}, kI {kI}, integralCap {integralCap},
    timeToMaxVoltage {timeToMaxVoltage},
    maxVoltage {(maxVoltage > 0 && maxVoltage <= 12 ? maxVoltage : 12)},
    profilePower {(profilePower >= 1 ? profilePower : 1)},
    startingVoltage {startingVoltage} {}

long double PIDController::getError() {
    return error;
}
long double PIDController::getDerivative() {
    return derivative;
}

void PIDController::setNewTarget(long double newTarget, bool ignoreProfile) {
    
    target      = newTarget;
    error       = 0;
    prevError   = 0;
    derivative  = 0;
    totalError  = 0;

    if (!ignoreProfile && timeToMaxVoltage > 0) {

        if (previousOutput > startingVoltage) {

            long double ceiling = maxVoltage - startingVoltage;
            if (previousOutput - startingVoltage >= ceiling) {
                return;
            }
            long double denominator = pow(ceiling / 2, profilePower - 1);

            usingProfile = true;

            if (previousOutput < ceiling / 2) {
                elapsedTime = pow(previousOutput * denominator, 1 / profilePower) * timeToMaxVoltage / ceiling;
            } else {
                elapsedTime = ceiling - pow((ceiling - previousOutput) * denominator, 1 / profilePower) * timeToMaxVoltage / ceiling;
            }

        }

    }

}

void PIDController::alterTarget(long double newTarget) {
    target = newTarget;
}

int PIDController::calcPower(long double currPos) {

    uint32_t currTime = pros::millis();
    long double dt = (currTime - startTime) / 1000.0L;
    if (dt == 0) {
        return previousOutput;
    }
    startTime = currTime;

    prevError = error;
    totalError += error * dt;

    long double integralTerm = kI * totalError;
    if (fabs(integralTerm) > integralCap) {
        integralTerm = integralCap * (integralTerm > 0 ? 1 : -1);
    }

    error = target - currPos;
    derivative = (error - prevError) / dt;
    long double pidOutput = kP * error + kD * derivative + integralTerm;

    if (fabs(pidOutput) > maxVoltage) {
        pidOutput = maxVoltage * (pidOutput > 0 ? 1 : -1);
    }

    if (usingProfile) {

        elapsedTime += dt;

        long double profileOutput;
        long double ceiling = maxVoltage - startingVoltage;
        long double denominator = pow(ceiling / 2, profilePower - 1);

        if (elapsedTime > timeToMaxVoltage / 2) {
            profileOutput = ceiling - pow(ceiling - ceiling * elapsedTime / timeToMaxVoltage, profilePower) / denominator;
        } else {
            profilePower = pow(ceiling * elapsedTime / timeToMaxVoltage, profilePower) / denominator;
        }

        profileOutput += startingVoltage;

        if (profileOutput > fabs(pidOutput) || elapsedTime > timeToMaxVoltage) {
            usingProfile = false;
        } else {
            pidOutput = profileOutput * (pidOutput > 0 ? 1 : -1);
        }

    }

    previousOutput = fabs(pidOutput);

    return pidOutput * 1000;

}

PIDController::Constants PIDController::getConstants() {
    return {
        kP, kD, kI, integralCap,
        timeToMaxVoltage, maxVoltage, profilePower, startingVoltage
    };
}

void PIDController::setConstants(long double newKP, long double newKD, long double newKI, long double newIntegralCap) {
    kP = newKP; kD = newKD; kI = newKI; integralCap = newIntegralCap;
}

void PIDController::setProfile(long double newTimeToMaxVoltage, long double newMaxVoltage,
    long double newProfilePower, long double newStartingVoltage)
{

    timeToMaxVoltage = newTimeToMaxVoltage;
    if (newTimeToMaxVoltage <= 0) {
        usingProfile = false;
    }

    maxVoltage = (newMaxVoltage > 0 && newMaxVoltage <= 12 ? newMaxVoltage : 12);
    profilePower = (newProfilePower >= 1 ? newProfilePower : 1);
    startingVoltage = newStartingVoltage;

}

} // namespace motor_control