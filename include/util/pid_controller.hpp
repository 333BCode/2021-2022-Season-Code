#ifndef _PID_HPP_
#define _PID_HPP_

#include <cstdint>

namespace motor_control {

    class PIDController final {
    public:

        struct Constants {

            const long double kP;
            const long double kD;
            const long double kI;
            const long double integralCap;

            const long double voltageAcceleration;
            const long double maxVoltage;
            const long double startingVoltage;

        };

        PIDController(long double kP, long double kD = 0, long double kI = 0, long double integralCap = 4,
            long double voltageAcceleration = 0, long double maxVoltage = 12, long double startingVoltage = 0);

        long double getError();
        long double getDerivative();

        void setNewTarget(long double newTarget, bool ignoreProfile = false);
        void alterTarget(long double newTarget);

        void updatePreviousSystemOutput(int previousSystemOutput);

        int calcPower(long double currPos);

        Constants getConstants();
        void setConstants(long double p, long double d, long double i, long double integralCap);
        void setSlewConstants(long double newVoltageAcceleration, long double newMaxVoltage, long double newStartingVoltage);

    private:

        long double kP;
        long double kD;
        long double kI;
        long double integralCap;

        long double target      {0};
        long double error       {0};
        long double prevError   {0};
        long double derivative  {0};
        long double totalError  {0};

        long double voltageAcceleration;
        long double maxVoltage;
        long double startingVoltage;
        long double slewPower   {0};
        uint32_t startTime      {0};

        bool usingSlew  {false};
        bool firstRun   {true};

        long double previousOutput {0};

    };

} // namespace motor_control

#endif