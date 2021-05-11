#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

class Drivetrain final {
public:

    class Point;

    enum class State {
        enabled,
        enabledStrafing
    };

    void operator()(const State& newState);
    static State getState();

    static std::array<long double, 3> getPosition();
    static void setPosition(long double newX, long double newY, long double newHeading);

    static void supply(int linearPow, int rotPow);
    static void supply(int linearPow, int strafePow, int rotPow);
    static void supplyVoltage(int linearPow, int rotPow);
    static void supplyVoltage(int linearPow, int strafePow, int rotPow);

    friend void mainTasks(void*);

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

    static const long double defaultLookAheadDistance;

    static const long double wheelSpacingParallel;
    static const long double wheelSpacingPerpendicular;
    static const long double trackingWheelDiameter;

    static long double ticksToInches(int ticks);

    static void trackPosition();

};

namespace drive {

    extern Drivetrain base;
    
    using State = Drivetrain::State;
    using Point = Drivetrain::Point;

}

class Drivetrain::Point final {
public:

    Point(long double x, long double y, long double heading = NAN);

    Point& withAction(std::function<void()>&& action, long double dist);
    Point& withLookAhead(long double newLookAhead);

private:

    struct Action final {

        Action(std::function<void()>&& newAction, long double dist);

        std::function<void()> action;
        long double distance;

    };

    long double x;
    long double y;
    long double heading;
    long double lookAheadDistance;
    std::vector<Action> actions;

};

#endif