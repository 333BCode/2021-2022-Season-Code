#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "util/pid_controller.hpp"

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

    static void stop(const pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_COAST);

    Drivetrain& operator<<(const Point& p);
    Drivetrain& operator>>(const Point& p);

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

    static long double oldTargetX;
    static long double oldTargetY;
    static long double targetHeading;

    static motor_control::PIDController linearPID;
    static motor_control::PIDController rotPID;

    static const long double defaultLookAheadDistance;

    static const long double wheelSpacingParallel;
    static const long double wheelSpacingPerpendicular;
    static const long double trackingWheelDiameter;

    static long double ticksToInches(int ticks);

    static void executeActions(const Point& p);

    static double distance(double dx, double dy);
    static long double wrapAngle(long double targetAngle);

    static int sign(long double num);

    static std::array<long double, 2> purePursuitLookAhead(
        long double lookAheadDistance,
        const std::array<long double, 2>& newEndpoint
    );

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

    Point& withAction(std::function<void()>&& action, double dist);
    Point& withLookAhead(long double newLookAhead);

    friend class Drivetrain;

private:

    struct Action final {

        Action(std::function<void()>&& newAction, double dist);

        std::function<void()> action;
        double distance;

    };

    long double x;
    long double y;
    long double heading;
    long double lookAheadDistance;
    std::vector<Action> actions;

};

#endif