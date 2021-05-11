#include "util/pid.hpp"
#include "pros/rtos.hpp"

#include <cmath>

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

    long double dt = pros::millis() - startTime;
    if (dt / 1000 == 0) {
        return previousOutput;
    }
    startTime += dt;
    dt /= 1000;

    prevError = error;
    totalError += error * dt;

    long double integralTerm = kI * totalError;
    if (fabs(integralTerm) > integralCap) {
        integralTerm = integralCap * (integralTerm > 0 ? 1 : -1);
    }

    error = target - currPos;
    long double pidOutput = kP * error + kD * (error - prevError) / dt + integralTerm;

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

    previousOutput = pidOutput;

    return static_cast<int>(pidOutput * 1000);

}

/*
long double kP;
long double kD;
long double kI;
long double integralCap;

long double target      {0};
long double error       {0};
long double prevError   {0};
long double derivative  {0};
long double totalError  {0};

long double timeToMaxVoltage;
long double maxVoltage;
long double profilePower;
long double startingVoltage;
uint32_t startTime      {0};

bool usingProfile {false};

int previousOutput {0};
*/

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

    maxVoltage = (newMaxVoltage > 0 && newMaxVoltage <= 12 ? newMaxVoltage : 12);
    profilePower = (newProfilePower >= 1 ? newProfilePower : 1);
    startingVoltage = newStartingVoltage;

}

} // namespace motor_control