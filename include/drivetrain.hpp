#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "util/pid_controller.hpp"
#include "api.h"
#include "macros.h"

#ifdef BRAIN_SCREEN_GAME_MODE
extern "C" void opcontrol();
#endif

class Drivetrain final {
public:

    class Point;
    struct XYPoint;
    struct XYHPoint;

    class Path;

    struct ExitConditions {

        long double maxLinearError;
        long double maxLinearDerivative;
        
        long double maxRotError;
        long double maxRotDerivative;

        long double minTime;
    
    };

    static bool isCalibrated();

    static XYHPoint getPosition();
    static void setPosition(long double newX, long double newY, long double newHeading);

    static void supply(int linearPow, int rotPow);
    static void supplyVoltage(int linearPow, int rotPow);

    Drivetrain& operator<<(const Point& p);
    Drivetrain& operator>>(const Point& p);

    static void turnTo(long double heading, const ExitConditions& exitConditions);
    static Point forward(long double dist);

    static const ExitConditions defaultExitConditions;   
    static void stop(const pros::motor_brake_mode_e_t brakeMode = pros::E_MOTOR_BRAKE_COAST);

    friend void mainTasks(void*);
#ifdef BRAIN_SCREEN_GAME_MODE
    friend void opcontrol();
#endif

private:

    static pros::Motor frontLeftMotor;
    static pros::Motor backLeftMotor;
    static pros::Motor frontRightMotor;
    static pros::Motor backRightMotor;

    static bool calibrated;

#ifdef USING_IMU
    static pros::Imu inertial;
#endif

    static pros::ADIEncoder leftEncoder;
    static pros::ADIEncoder rightEncoder;
    static pros::ADIEncoder middleEncoder;

    static pros::Mutex positionDataMutex;
    static pros::Mutex calibrationMutex;

    static long double xPos;
    static long double yPos;
    static long double heading;

    static long double oldTargetX;
    static long double oldTargetY;
    static long double targetHeading;

    static motor_control::PIDController linearPID;
    static motor_control::PIDController rotPID;

    static bool stopped;

    static const long double defaultLookAheadDistance;
    static const long double minDistForTurning;

    static const long double wheelSpacingParallel;
    static const long double wheelSpacingPerpendicular;
    static const long double trackingWheelDiameter;

    static long double maxVelocity;
    static long double maxAcceleration;
    static long double drivetrainWidth;
    static long double profileDT;

    static long double ticksToInches(int ticks);

    static void executeActions(const Point& p);

    static long double wrapAngle(long double targetAngle);

    static int sign(long double num);

    static XYPoint purePursuitLookAhead(
        long double lookAheadDistance,
        XYPoint newEndpoint
    );

    static void trackPosition();

};

namespace drive {

    extern Drivetrain base;

    extern Drivetrain::Point (*const forward)(long double);
    
    using Point     = Drivetrain::Point;
    using XYPoint   = Drivetrain::XYPoint;
    using XYHPoint  = Drivetrain::XYHPoint;

    using Path = Drivetrain::Path;

    using ExitConditions = Drivetrain::ExitConditions;

}

#include "drivetrain/point.hpp"
#include "drivetrain/path.hpp"

#endif