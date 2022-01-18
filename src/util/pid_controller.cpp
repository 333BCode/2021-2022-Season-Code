#include "util/pid_controller.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace motor_control {

    PIDController::PIDController(long double kP, long double kD, long double kI, long double integralCap,
        long double voltageAcceleration, long double maxVoltage, long double startingVoltage)
        : kP {kP}, kD {kD}, kI {kI}, integralCap {integralCap},
        voltageAcceleration {voltageAcceleration},
        maxVoltage {(maxVoltage > 0 && maxVoltage <= 12 ? maxVoltage : 12)},
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

        firstRun = true;

        usingSlew = (!ignoreProfile && voltageAcceleration > 0);
        slewPower = previousOutput;

    }

    void PIDController::alterTarget(long double newTarget) {
        target = newTarget;
    }

    void PIDController::updatePreviousSystemOutput(int previousSystemOutput) {
        previousOutput = previousSystemOutput / 1000.0;
    }

    int PIDController::calcPower(long double currPos) {

        static bool positiveSlewAcceleration = true;

        uint32_t currTime = pros::millis();
        long double dt = (currTime - startTime) / 1000.0L;
        if (firstRun) {
            dt = 0.01;
            if (usingSlew) {
                error = target - currPos;
                positiveSlewAcceleration = (error >= 0);
                slewPower += startingVoltage * (positiveSlewAcceleration ? 1 : -1);
            }
            firstRun = false;
        } else if (dt == 0) {
            return previousOutput;
        }
        startTime = currTime;

        prevError = error;
        totalError += error * dt;

        long double integralTerm = std::clamp(kI * totalError, -integralCap, integralCap);

        error = target - currPos;
        derivative = (error - prevError) / dt;
        long double pidOutput = std::clamp(kP * error + kD * derivative + integralTerm, -maxVoltage, maxVoltage);

        if (usingSlew) {

            slewPower += voltageAcceleration * dt * (positiveSlewAcceleration ? 1 : -1);

            if (fabs(slewPower) >= fabs(pidOutput)) {
                usingSlew = false;
            } else {
                pidOutput = slewPower;
            }

        }

        previousOutput = pidOutput;

        return pidOutput * 1000;

    }

    PIDController::Constants PIDController::getConstants() {
        return {
            kP, kD, kI, integralCap,
            voltageAcceleration, maxVoltage, startingVoltage
        };
    }

    void PIDController::setConstants(long double newKP, long double newKD, long double newKI, long double newIntegralCap) {
        kP = newKP; kD = newKD; kI = newKI; integralCap = newIntegralCap;
    }

    void PIDController::setSlewConstants(long double newVoltageAcceleration, long double newMaxVoltage, long double newStartingVoltage)
    {

        voltageAcceleration = newVoltageAcceleration;
        if (voltageAcceleration <= 0) {
            usingSlew = false;
        }

        maxVoltage = (newMaxVoltage > 0 && newMaxVoltage <= 12 ? newMaxVoltage : 12);
        startingVoltage = newStartingVoltage;

    }

} // namespace motor_control