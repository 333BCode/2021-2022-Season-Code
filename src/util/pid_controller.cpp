#include "util/pid_controller.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace motor_control {

    // constructor
    // note the derivative gain (kD) comes before the integral gain (kI)
    // integral cap limits the max voltage that the integral term can supply
    // the last three parameters dictate slew control, and have units of volts/s^s, volts, and volts respectively
    // if voltageAcceleration <= 0, slew control will not be used
    PIDController::PIDController(long double kP, long double kD, long double kI, long double integralCap,
        long double voltageAcceleration, long double maxVoltage, long double startingVoltage)
        : kP {kP}, kD {kD}, kI {kI}, integralCap {integralCap},
        voltageAcceleration {voltageAcceleration},
        maxVoltage {(maxVoltage > 0 && maxVoltage <= 12 ? maxVoltage : 12)},
        startingVoltage {startingVoltage} {}

    // returns the error of the system when calcPower was last run
    long double PIDController::getError() {
        return error;
    }

    // returns the derivative of the system when calc power was last run
    long double PIDController::getDerivative() {
        return derivative;
    }

    // resets the PIDController and sets a new target
    // if ignoreProfile is true, slew is disabled until the next reset, otherwise slew is enabled (if voltageAcceleration > 0)
    void PIDController::setNewTarget(long double newTarget, bool ignoreProfile) {
        
        // reset state variables
        target      = newTarget;
        error       = 0;
        prevError   = 0;
        derivative  = 0;
        totalError  = 0;

        // mark the next calcPower call as the first for the motion
        firstRun = true;

        // update the slew profile by conditionally enabling it and updating the initial slew output
        usingSlew = (!ignoreProfile && voltageAcceleration > 0);
        slewPower = previousOutput;

    }

    // changes the target, does not reset slew, derivative, or integral terms
    void PIDController::alterTarget(long double newTarget) {
        target = newTarget;
    }

    // Inform the PIDController of updates to the system's output since calcPower was last ran
    // (example: after a motion that does not utilize PID)
    // units are millivolts for consistency with calcPower
    void PIDController::updatePreviousSystemOutput(int previousSystemOutput) {
        previousOutput = previousSystemOutput / 1000.0; // convert from millivolts to volts
    }

    // returns a suggestion for the millivolts to supply to the system, based off of the current position of said system
    // units are millivolts because pros::Motor::move_voltage uses millivolts
    int PIDController::calcPower(long double currPos) {

        // state variable to track the direction of slew acceleration
        static bool positiveSlewAcceleration = true;

        uint32_t currTime = pros::millis();
        long double dt = (currTime - startTime) / 1000.0L; // important for slew, integral, and derivative terms
        if (firstRun) {
            // Update slew starting power and direction at the start of a new motion, prevent dt from
            // causing the calculations to be skipped and the previous output (which is not reset) to be returned
            // This also causes error to be properly tracked on motion start if dt is 0
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

        // update state variables for next function call
        startTime = currTime;
        prevError = error;

        // integrate, find and limit integral term
        totalError += error * dt;
        long double integralTerm = std::clamp(kI * totalError, -integralCap, integralCap);

        // find error, derivative, sum up weighted P, I, and D and bound the result
        error = target - currPos;
        derivative = (error - prevError) / dt;
        long double pidOutput = std::clamp(kP * error + kD * derivative + integralTerm, -maxVoltage, maxVoltage);

        if (usingSlew) { // if motion profiling the beginning of a motion

            // apply acceleration to the suggested power
            slewPower += voltageAcceleration * dt * (positiveSlewAcceleration ? 1 : -1);

            if (fabs(slewPower) >= fabs(pidOutput)) { // stop using slew once past PID output
                usingSlew = false;
            } else { // replace PID output with slew output
                pidOutput = slewPower;
            }

        }

        // store the output
        previousOutput = pidOutput;

        return pidOutput * 1000; // convert from volts to millivolts

    }

    // returns the current gains for both PID and slew
    PIDController::Constants PIDController::getConstants() {
        return {
            kP, kD, kI, integralCap,
            voltageAcceleration, maxVoltage, startingVoltage
        };
    }

    // sets the PID gains
    // note the derivative gain (newKD) comes before the integral gain (newKI)
    void PIDController::setConstants(long double newKP, long double newKD, long double newKI, long double newIntegralCap) {
        kP = newKP; kD = newKD; kI = newKI; integralCap = newIntegralCap;
    }

    // sets the slew constants with units of volts/s^s, volts, and volts respectively
    // if voltageAcceleration <= 0, slew control will not be used
    void PIDController::setSlewConstants(
        long double newVoltageAcceleration, long double newMaxVoltage, long double newStartingVoltage
    ) {

        voltageAcceleration = newVoltageAcceleration;
        if (voltageAcceleration <= 0) { // if not a vaild acceleration, do not use slew
            usingSlew = false;
        }

        // set maxVoltage to a value on the interval (0, 12]
        maxVoltage = (newMaxVoltage > 0 && newMaxVoltage <= 12 ? newMaxVoltage : 12);
        startingVoltage = newStartingVoltage;

    }

} // namespace motor_control