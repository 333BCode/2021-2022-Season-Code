#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "util/pid_controller.hpp"
#include "api.h"
#include "macros.h"

class Drivetrain final {
public:

    struct XYPoint;
    struct Point;
    struct Waypoint;

    class Path;

    struct ExitConditions {

        long double maxLinearError;
        long double maxLinearDerivative;
        
        long double maxRotError;
        long double maxRotDerivative;

        long double minTime;
    
    };

    static void waitUntilCalibrated();
    static void setReversed(bool reversed);

    static Point getPosition();
    static void setPosition(long double newX, long double newY, long double newHeading);

    static void supply(int linearPow, int rotPow);
    static void supplyVoltage(int linearPow, int rotPow);
    static void supplyVoltagePerSide(int leftVoltage, int rightVoltage);

    static const ExitConditions defaultExitConditions;

    Drivetrain& operator<<(const Path& path);
    Drivetrain& operator<<(const Waypoint& p);
    Drivetrain& operator>>(Point p);
    static void moveTo(
        long double x, long double y,
        long double heading = NAN, const ExitConditions& exitConditions = defaultExitConditions
    );
    static void moveTo(long double x, long double y, const ExitConditions& exitConditions);
    static void turnTo(long double heading, const ExitConditions& exitConditions = defaultExitConditions);
    static void moveForward(long double dist);

    static void addAction(std::function<void()>&& action, double dist, bool duringTurn = false);
  
    static void stopMotion();
    static void setBrakeMode(const pros::motor_brake_mode_e_t brakeMode);

    friend void mainTasks(void*);

private:

    struct Action final {

        Action(std::function<void()>&& newAction, double atError, bool duringTurn = false);

        std::function<void()> action;
        double error;
        bool duringTurn;

    };

    static pros::Motor frontLeftMotor;
    static pros::Motor backLeftMotor;
    static pros::Motor frontRightMotor;
    static pros::Motor backRightMotor;

    static bool calibrated;
    static bool driveReversed;

    static pros::Imu inertial;

    static pros::ADIEncoder leftEncoder;
    // static pros::ADIEncoder rightEncoder;
    static pros::ADIEncoder middleEncoder;

    static pros::Mutex positionDataMutex;

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
    static const long double drivetrainWidth;
    static long double profileDT;

    static std::vector<Action> actionList;

    static long double ticksToInches(int ticks);

    static void executeActions(double currError, bool inTurn = false);

    static void endMotion();
    static void endMotion(long double targetX, long double targetY);

    static long double wrapAngle(long double targetAngle);

    static int sign(long double num);

    static XYPoint purePursuitLookAhead(
        long double lookAheadDistance,
        XYPoint newEndpoint
    );

    static void trackPosition();

};

#include "drivetrain/point.hpp"
#include "drivetrain/path.hpp"

namespace drive {

    extern Drivetrain base;
    
    using XYPoint   = Drivetrain::XYPoint;
    using Point     = Drivetrain::Point;
    using Waypoint  = Drivetrain::Waypoint;

    using Path = Drivetrain::Path;

    using ExitConditions = Drivetrain::ExitConditions;

    extern Drivetrain::Path (*const generatePathTo)(Drivetrain::Point);
    extern Drivetrain::Path (*const generatePath)(Drivetrain::Point, Drivetrain::Point);

} // namespace drive

#endif