#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

class Drivetrain final {
public:

    enum class State {
        enabled,
        enabledStrafing
    };

    void operator()(const State& newState);
    static State getState();

    static std::array<int, 3> getPosition();
    static void setPosition(long double newX, long double newY, long double newHeading);

    static void supply(int linearPow, int rotPow);
    static void supply(int linearPow, int strafePow, int rotPow);
    static void supplyVoltage(int linearPow, int rotPow);
    static void supplyVoltage(int linearPow, int strafePow, int rotPow);

    friend void mainTasks();

private:

    static pros::Motor frontLeftMotor;
    static pros::Motor backLeftMotor;
    static pros::Motor frontRightMotor;
    static pros::Motor backRightMotor;

    static pros::Imu inertial;

    static pros::ADIEncoder leftEncoder;
    static pros::ADIEncoder rightEncoder;
    static pros::ADIEncoder middleEncoder;

    static State state;

    static pros::Mutex positionDataMutex;

    static long double xPos;
    static long double yPos;
    static long double heading;

    static const long double wheelSpacingParallel;
    static const long double wheelSpacingPerpendicular;
    static const long double trackingWheelDiameter;

    static long double ticksToInches(int ticks);

    static void trackPosition();

};

extern Drivetrain base;

namespace drive {
    using State = Drivetrain::State;
}

#endif