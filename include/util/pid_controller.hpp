#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

namespace motor_control {

class PIDController final {
public:

    struct Constants {

        const long double kP;
        const long double kD;
        const long double kI;
        const long double integralCap;

        const long double timeToMaxVoltage;
        const long double maxVoltage;
        const long double profilePower;
        const long double startingVoltage;

    };

    PIDController(long double kP, long double kD = 0, long double kI = 0, long double integralCap = 4,
        long double timeToMaxVoltage = 0, long double maxVoltage = 12, long double profilePower = 1, long double startingVoltage = 0);

    long double getError();
    long double getDerivative();

    void setNewTarget(long double newTarget);
    void alterTarget(long double newTarget);

    int calcPower(long double currPos);

    Constants getConstants();
    void setConstants(long double p, long double d, long double i, long double integralCap);
    void setProfile(long double newTimeToMaxVoltage, long double newMaxVoltage,
        long double newProfilePower, long double newStartingVoltage);

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

    long double timeToMaxVoltage;
    long double maxVoltage;
    long double profilePower;
    long double startingVoltage;
    uint32_t startTime      {0};
    long double elapsedTime {0};

    bool usingProfile {false};

    long double previousOutput {0};

};

} // namespace motor_control

#endif