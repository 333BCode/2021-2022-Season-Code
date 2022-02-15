#ifndef _PID_HPP_
#define _PID_HPP_

#include <cstdint>

/**
 * This file contains the declaration of a PID controller
 *
 * PID stands for proportional-integral-derivative (all taken with respect to error in position)
 * PID is used to accurately settle systems at target positions
 * PIDController does this by providing a suggestion of what voltage the motors should run at
 *
 * Drivetrain uses two PID controllers, one for distance from the target and one for heading
 * Lift uses the motor's integrated PID control; this is accurate for our purposes,
 * so there is no need to use this PIDController for Lift
 *
 * PIDController also implements slew control, which is a one dimensional trapezoidal motion profile for the start of motions
 * Without slew control, at the start of most motions (new requested target state)
 * PIDController will immediately suggest that the motors run at max voltage
 *
 * Unless otherwise noted, units used for voltage are volts
 */

namespace motor_control {

    class PIDController final {
    public:

        // struct to contain current PID and slew constants
        struct Constants {

            const long double kP;
            const long double kD;
            const long double kI;
            const long double integralCap;

            const long double voltageAcceleration;
            const long double maxVoltage;
            const long double startingVoltage;

        };

        // constructor
        // note the derivative gain (kD) comes before the integral gain (kI)
        // integral cap limits the max voltage that the integral term can supply
        // the last three parameters dictate slew control, and have units of volts/s^s, volts, and volts respectively
        // if voltageAcceleration <= 0, slew control will not be used
        PIDController(long double kP, long double kD = 0, long double kI = 0, long double integralCap = 4,
            long double voltageAcceleration = 0, long double maxVoltage = 12, long double startingVoltage = 0);

        // returns the error of the system when calcPower was last run
        long double getError();
        // returns the derivative of the system when calc power was last run
        long double getDerivative();

        // resets the PIDController and sets a new target
        // if ignoreProfile is true, slew is disabled until the next reset, otherwise slew is enabled (if voltageAcceleration > 0)
        void setNewTarget(long double newTarget, bool ignoreProfile = false);
        // changes the target, does not reset slew, derivative, or integral terms
        void alterTarget(long double newTarget);

        // inform the PIDController of updates to the system's output since calcPower was last ran (example: after a motion that does not utilize PID)
        // units are millivolts for consistency with calcPower
        void updatePreviousSystemOutput(int previousSystemOutput);

        // returns a suggestion for the millivolts to supply to the system, based off of the current position of said system
        // units are millivolts because pros::Motor::move_voltage uses millivolts
        int calcPower(long double currPos);

        // returns the current gains for both PID and slew
        Constants getConstants();
        // sets the PID gains
        // note the derivative gain (d) comes before the integral gain (i)
        void setConstants(long double p, long double d, long double i, long double integralCap);
        // sets the slew constants with units of volts/s^s, volts, and volts respectively
        // if voltageAcceleration <= 0, slew control will not be used
        void setSlewConstants(long double newVoltageAcceleration, long double newMaxVoltage, long double newStartingVoltage);

    private:

        // PID gains
        long double kP;
        long double kD;
        long double kI;
        long double integralCap;

        // positional data for the system
        long double target      {0};
        long double error       {0};
        long double prevError   {0};
        long double derivative  {0};
        long double totalError  {0};

        // slew constants
        long double voltageAcceleration;
        long double maxVoltage;
        long double startingVoltage;
        // stores the current power to supply due to slew control
        long double slewPower   {0};
        // stores the time at which calcPower was last run (for derivative, integral, and slew calculations)
        uint32_t startTime      {0};

        // state of whether slew control should be used / calculated
        bool usingSlew  {false};
        // state of whether calcPower is being called for the first time after a reset
        // When true, the slew profile is given direction,
        // and the previous output will not override the output (which would normally happen if dT between calcPower calls is 0ms)
        bool firstRun   {true};

        // stores the value of the previous output from the system (either from calcPower or from information supplied by updatePreviousSystemOutput)
        // used when dT between calcPower calls is 0ms and to properly reset the slew profile during a setNewTarget call
        long double previousOutput {0};

    };

} // namespace motor_control

#endif
