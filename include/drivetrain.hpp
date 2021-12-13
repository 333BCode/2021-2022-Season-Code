#ifndef _DRIVETRAIN_HPP_
#define _DRIVETRAIN_HPP_

#include "util/pid_controller.hpp"
#include "api.h"
#include "macros.h"

/**
 * Declaration for the Drivetrain class and the drive namespace
 *
 * Drivetrain is used to abstractly supply power to the drivetrain motors, both in driver control and auton
 *
 * Drivetrain is a static class, however there are a few non static operator overloads
 * An instance of Drivetrain, base, is found in the drive namespace
 *
 * Drivetrain implements several algorithms to ensure accurate and efficient autonomous movement:
 *      Odometry: tracks position for mid motion error correction, is always running
 *      Pure Pursuit: smooths motions when transversing several Waypoint
 *      Motion Profiling: a feedforward algorithm to maximize efficiency, uses feedback for additional error correction
 *
 * Drivetrain utilizes two PID_Controllers, one for linear motion and one for rotational motion,
 * to power the motors to accurately reach our targets
 *
 * Values passed to Drivetrain methods have units of inches and degrees
 * The literals and functions in util/conversions.hpp follow these unit defaults
 *
 * For the purposes of Drivetrain, some methods allow the user to specify whether a movement is absolute of not:
 *      Absolute: the final state is based off of where the bot was prior told to go
 *      Relative (not absolute): the final state is based off of where the bot starts the motion
 */

class Drivetrain final {
public:

    /**
     * Point types
     */

    struct XYPoint;
    struct Point;
    struct Waypoint;

    /**
     * Motion profile data storage
     */

    class Path;

    /**
     * Function signatures for exit conditions
     */

    typedef bool (*PurePursuitExitConditions)(long double, long double, bool);
    typedef bool (*LinearExitConditions)(long double, bool, bool);
    typedef bool (*TurnExitConditions)(bool, bool);

    // includes templated (with default template arguments) default exit condition functions
#include "drivetrain/exit_conditions.hpp"

    // Blocks task until the Drivetrain IMU is calibrated (odom can start running)
    static void waitUntilCalibrated();

    // enum class for the directions the robot can follow motions in
    enum class Direction {
        autoDetermine, // determine which direction a motion will be followed at the start of the motion, uses absolute coordinates
        forward,
        reverse
    };

    // Set the direction in which the Drivetrain will follow motions
    static void setFollowDirection(Direction direction);

    // Returns the tracked position
    // MUTEX LOCKING
    static Point getPosition();
    // Sets the tracked position (use to tell the Drivetrain where it is)
    // MUTEX LOCKING
    static void setPosition(long double newX, long double newY, long double newHeading);

    // Supply power to the Drivetrain motors
    // Forward and clockwise (due to controller joystick notation) are positive
    static void supply(int linearPow, int rotPow);
    static void supplyVoltage(int linearPow, int rotPow);

    /**
     * Movement functions
     *
     * These execute precise movements using a variety of algorithms
     * All movements use real time position data (as determined by odometry) for mid motion feedback error correction
     *
     * ALL Movement methods are MUTEX LOCKING
     */

    // Follows the motion profile, stored in the Path, uses feedback error correction during the motion
    Drivetrain& operator<<(const Path& path);
    // Uses pure pursuit to move towards the Waypoint; for optimal use utilize several pure pursuit movements in succession
    Drivetrain& operator<<(const Waypoint& p);
    // Invokes moveTo to move to the Point
    Drivetrain& operator>>(Point p);

    // If heading is a number (is specified), will invoke turnTo after reaching the desired position
    static void moveTo(
        long double x, long double y, long double heading = NAN,
        LinearExitConditions linearExitConditions = defaultLinearExit, TurnExitConditions turnExitConditions = defaultTurnExit
    );
    // Will turn to face targetForHeading using absolute coordinates after reaching the desired position
    static void moveTo(
        long double x, long double y, XYPoint targetForHeading,
        LinearExitConditions linearExitConditions = defaultLinearExit, TurnExitConditions turnExitConditions = defaultTurnExit
    );
    // Will move to the desired position
    static void moveTo(long double x, long double y, LinearExitConditions linearExitConditions);

    // Will turn to the desired heading
    static void turnTo(long double heading, TurnExitConditions exitConditions = defaultTurnExit);
    // Will turn to face target using the system specified by the bool absolute
    static void turnTo(XYPoint target, bool absolute = false, TurnExitConditions exitConditions = defaultTurnExit);

    // Moves forward, uses the system specified by the bool absolute to determine the desired final position
    // Does not invoke turnTo after the movement is complete
    static void moveForward(long double dist, bool absolute = true, LinearExitConditions exitConditions = defaultLinearExit);

    /**
     * End of movement functions
     */
    
    // Store an action to be executed during the next movement at the given error
    static void addAction(std::function<void()>&& action, double dist, bool duringTurn = false);

    // Stops a motion early when called during that motion (pass stopMotion to addAction)
    static void stopMotion();
    // Sets the brake mode of the motors
    static void setBrakeMode(const pros::motor_brake_mode_e_t brakeMode);

    // Allows mainTasks to calibrate IMU, reset tracking wheel encoders, mark when calibration is complete, and run odometry
    friend void mainTasks(void*);

private:

    // Struct to store actions to preform (functions to call) during a motion
    struct Action final {

        // constructor
        Action(std::function<void()>&& newAction, double atError, bool duringTurn = false);

        // action to execute
        std::function<void()> action;
        // error at which to execute the action
        double error;
        // execute during a linear movement or turn (needed for moveTo commands which invoke turnTo)
        bool duringTurn;

    };

    static pros::Motor frontLeftMotor;
    static pros::Motor backLeftMotor;
    static pros::Motor frontRightMotor;
    static pros::Motor backRightMotor;

    static bool calibrated;
    static bool autoDetermineReversed;
    static bool driveReversed;

    static pros::Imu inertial;

    static pros::ADIEncoder rightEncoder;
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

    static void determineFollowDirection(long double xTarget, long double yTarget);

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

    using Direction = Drivetrain::Direction;

    using PurePursuitExitConditions = Drivetrain::PurePursuitExitConditions;
    using LinearExitConditions = Drivetrain::LinearExitConditions;
    using TurnExitConditions = Drivetrain::TurnExitConditions;

    extern Drivetrain::Path (*const generatePathTo)(Drivetrain::Point);
    extern Drivetrain::Path (*const generatePath)(Drivetrain::Point, Drivetrain::Point);

} // namespace drive

#endif