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
 *
 * Drivetrain uses a mutex to protect its static member variables from data races (between the field control and main threads)
 * Functions that lock the internal mutex are labeled: MUTEX LOCKING
 * These functions can be freely used where there is not access to private members of Drivetrain (like when writing autons)
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
    // In a separate file to save space / readability in this one
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
     */

    // Follows the motion profile, stored in the Path, uses feedback error correction during the motion
    // MUTEX LOCKING
    Drivetrain& operator<<(const Path& path);
    // Uses pure pursuit to move towards the Waypoint; for optimal use utilize several pure pursuit movements in succession
    // MUTEX LOCKING
    Drivetrain& operator<<(const Waypoint& p);
    // Invokes moveTo to move to the Point
    // MUTEX LOCKING
    Drivetrain& operator>>(Point p);

    // If heading is a number (is specified), will invoke turnTo after reaching the desired position
    // MUTEX LOCKING
    static void moveTo(
        long double x, long double y, long double heading = NAN,
        LinearExitConditions linearExitConditions = defaultLinearExit, TurnExitConditions turnExitConditions = defaultTurnExit
    );
    // Will turn to face targetForHeading using absolute coordinates after reaching the desired position
    // MUTEX LOCKING
    static void moveTo(
        long double x, long double y, XYPoint targetForHeading,
        LinearExitConditions linearExitConditions = defaultLinearExit, TurnExitConditions turnExitConditions = defaultTurnExit
    );
    // Will move to the desired position
    // MUTEX LOCKING
    static void moveTo(long double x, long double y, LinearExitConditions linearExitConditions);

    // Will turn to the desired heading
    // MUTEX LOCKING
    static void turnTo(long double heading, TurnExitConditions exitConditions = defaultTurnExit);
    // Will turn to face target using the system specified by the bool absolute
    // MUTEX LOCKING
    static void turnTo(XYPoint target, bool absolute = false, TurnExitConditions exitConditions = defaultTurnExit);

    // Moves forward, uses the system specified by the bool absolute to determine the desired final position
    // Does not invoke turnTo after the movement is complete
    // MUTEX LOCKING
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

    static void limitSpeed(long double speed);
    static void limitLinearSpeed(long double speed);
    static void limitTurnSpeed(int voltage);
    static void unboundSpeed();
    static void unboundLinearSpeed();
    static void unboundTurnSpeed();
    static void setLinearSlew(int slewPower);
    static const long double maxVelocity;

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

    /**
     * Devices
     *
     * All devices are instantiated in src/devices.cpp
     */

    /* motors: used in field control tasks */

    static pros::Motor frontLeftMotor;
    static pros::Motor topBackLeftMotor;
    static pros::Motor bottomBackLeftMotor;
    static pros::Motor frontRightMotor;
    static pros::Motor topBackRightMotor;
    static pros::Motor bottomBackRightMotor;

    /* sensors: used in main tasks */

    static pros::Imu imu1;
    static pros::Imu imu2;

    static pros::Rotation parallelTrackingWheel;
    static pros::ADIEncoder perpendicularTrackingWheel;

    // Mutex: protects positional data, instantiated in src/drivetrain/drivetrain.cpp
    static pros::Mutex positionDataMutex;

    /**
     * Positional Data
     *
     * Instantiated in src/drivetrain/drivetrain.cpp
     */
    
    /* current position: used in field control and main tasks */

    // NEEDS MUTEX COVER
    static long double xPos;
    // NEEDS MUTEX COVER
    static long double yPos;
    // NEEDS MUTEX COVER
    static long double heading;

    /* old targeted position: used in main task */

    // Used to implement segmented pure pursuit
    static long double oldTargetX;
    // Used to implement segmented pure pursuit
    static long double oldTargetY;
    // Used to keep a consistent heading between motions (that may not specify a target heading)
    static long double targetHeading;

    /**
     * PID Controllers
     *
     * Instantiated in src/drivetrain/constants.cpp
     *
     * Used in field control tasks
     */

    static motor_control::PIDController linearPID;
    static motor_control::PIDController rotPID;

    /**
     * State data
     *
     * Instantiated in src/drivetrain.drivetrain.cpp
     */

    // NEEDS MUTEX COVER: used in field control and main tasks
    static bool calibrated;

    /* Used in field control tasks */

    static bool autoDetermineReversed;
    static bool driveReversed;

    static bool stopped;

    /**
     * Constants
     *
     * Instantiated in src/drivetrain/constants.cpp
     */

    // Used to implement pure pursuit, used in field control tasks
    static const long double defaultLookAheadDistance;
    // Used to implement non linear moveTo functionality, used in field control tasks
    static const long double minDistForTurning;

    /* odometry constants: used in main task */

    static const long double wheelSpacingParallel;
    static const long double wheelSpacingPerpendicular;
    static const long double trackingWheelDiameter;

    static int linearSpeedLimit;
    static int rotSpeedLimit;

    /* motion profiling constants: used in field control tasks */

    static const long double maxAcceleration;
    static const long double drivetrainWidth;
    // Time step used when generating a profile, should equal the time delay used in the loop that runs the profile
    static const long double profileDT;

    // Storage for actions to execute mid motion: used in field control tasks
    static std::vector<Action> actionList;

    /**
     * Private movement functions
     *
     * Used in field control tasks
     */

    // Updates driveReversed if autoDetermineReversed
    static void determineFollowDirection(long double xTarget, long double yTarget);

    // Converts encoder rotations to inches traveled
    static long double ticksToInches(int ticks);

    // Executes actions stored in actionList if they are eligible to be executed
    static void executeActions(double currError, bool inTurn = false);

    // Clears actionList, sets motor power to 0
    static void endMotion();
    // Clears actionList, sets motor power to 0, updates old target variables
    static void endMotion(long double targetX, long double targetY);

    // Converts the heading based off of targetAngle to minimize the distance between the two
    // NEEDS MUTEX COVER: accesses positional data
    static long double wrapAngle(long double targetAngle);

    // Returns 1 if positive or 0, -1 if negative
    static int sign(long double num);

    // Returns the point for the movement algorithm to target, as determined by pure pursuit
    static XYPoint purePursuitLookAhead(
        long double lookAheadDistance,
        XYPoint newEndpoint
    );

    // Carry out one step of odometry calculations, called in main task
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