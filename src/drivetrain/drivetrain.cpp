#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "util/conversions.hpp"
#include "macros.h"

namespace drive {

    // instance of Drivetrain
    Drivetrain base {};

} // namespace drive

/**
 * State and position data initilization
 */
bool Drivetrain::calibrated             = false;
bool Drivetrain::autoDetermineReversed  = false;
bool Drivetrain::driveReversed          = false;

bool Drivetrain::stopped = false;

int Drivetrain::linearSpeedLimit    = 12000;
int Drivetrain::rotSpeedLimit       = 12000;

pros::Mutex Drivetrain::positionDataMutex {};

/**
 * Immediate position initilization is irrelevant, as autons should call Drivetrain::setPosition at the start
 *
 * Initialized to middle of the field
 */
long double Drivetrain::xPos    = 72;
long double Drivetrain::yPos    = 72;
long double Drivetrain::heading = 90;

long double Drivetrain::oldTargetX      = 72;
long double Drivetrain::oldTargetY      = 72;
long double Drivetrain::targetHeading   = 90;

// Storage for actions to execute mid motion
std::vector<Drivetrain::Action> Drivetrain::actionList {};

// Blocks task until the Drivetrain IMU is calibrated (odom can start running)
void Drivetrain::waitUntilCalibrated() {
    while (true) {
    positionDataMutex.take();
        bool isCalibrated = calibrated;
    positionDataMutex.give();
        if (isCalibrated) {
            break;
        }
        pros::delay(10); // delay task
    }
}

// Set the direction in which the Drivetrain will follow motions
void Drivetrain::setFollowDirection(Direction direction) {
    switch (direction) {
        case Direction::autoDetermine:
            autoDetermineReversed = true;
        break;
        case Direction::forward:
            autoDetermineReversed = false;
            driveReversed = false;
        break;
        case Direction::reverse:
            autoDetermineReversed = false;
            driveReversed = true;
        break;
    }
}

/**
 * Limits the voltage supplied to the motors during a motion
 * This is used to control top speed
 * Unless if otherwise noted, speed is in inches per second
 */

// Limits linear and turn speeds
void Drivetrain::limitSpeed(long double speed) {
    linearSpeedLimit    = 12000 * speed / maxVelocity; // convert velocity to respective voltage
    rotSpeedLimit       = linearSpeedLimit;
}

void Drivetrain::limitLinearSpeed(long double speed) {
    linearSpeedLimit = 12000 * speed / maxVelocity; // convert velocity to respective voltage
}

// voltage is in millivolts (out of 12000)
void Drivetrain::limitTurnSpeed(int voltage) {
    rotSpeedLimit = voltage;
}

// Unbounds linear and turn speeds
void Drivetrain::unboundSpeed() {
    linearSpeedLimit    = 12000;
    rotSpeedLimit       = 12000;
}

void Drivetrain::unboundLinearSpeed() {
    linearSpeedLimit = 12000;
}

void Drivetrain::unboundTurnSpeed() {
    rotSpeedLimit = 12000;
}

// When called between motions, sets the starting voltage of the linearPID slew controller during the next motion
void Drivetrain::setLinearSlew(int slewPower) {
    linearPID.updatePreviousSystemOutput(slewPower);
}

// Returns the tracked position
Drivetrain::Point Drivetrain::getPosition() {
positionDataMutex.take(20); // timeout and prevent deadlock if other task exits without freeing the mutex
    Point position = {xPos, yPos, heading};
positionDataMutex.give();
    return position;
}

// Sets the tracked position (use to tell the Drivetrain where it is)
void Drivetrain::setPosition(long double newX, long double newY, long double newHeading) {
positionDataMutex.take();
    xPos = newX; yPos = newY;
    // wrap heading to be on the interval [0, 360)
    if (newHeading >= 360) {
        heading = newHeading - 360;
    } else if (newHeading < 0) {
        heading = newHeading + 360;
    } else {
        heading = newHeading;
    }
    // update old targets so pure pursuit and moveForward commands function properly
    oldTargetX = newX; oldTargetY = newY; targetHeading = heading;
positionDataMutex.give();
}

// Supply power to the Drivetrain motors [-127, 127]
// Forward and clockwise (due to controller joystick notation) are positive
void Drivetrain::supply(int linearPow, int rotPow) {
    frontLeftMotor.move(linearPow + rotPow);
    topBackLeftMotor.move(linearPow + rotPow);
    bottomBackLeftMotor.move(linearPow + rotPow);
    frontRightMotor.move(linearPow - rotPow);
    topBackRightMotor.move(linearPow - rotPow);
    bottomBackRightMotor.move(linearPow - rotPow);
}

// Supply power to the Drivetrain motors [-12000, 12000]
// Forward and clockwise (due to controller joystick notation) are positive
void Drivetrain::supplyVoltage(int linearPow, int rotPow) {
    frontLeftMotor.move_voltage(linearPow + rotPow);
    topBackLeftMotor.move_voltage(linearPow + rotPow);
    bottomBackLeftMotor.move_voltage(linearPow + rotPow);
    frontRightMotor.move_voltage(linearPow - rotPow);
    topBackRightMotor.move_voltage(linearPow - rotPow);
    bottomBackRightMotor.move_voltage(linearPow - rotPow);
}

// Stops a motion early when called during that motion (pass stopMotion to addAction)
void Drivetrain::stopMotion() {
    stopped = true;
}

// Sets the brake mode of the motors
void Drivetrain::setBrakeMode(const pros::motor_brake_mode_e_t brakeMode) {
    frontLeftMotor.set_brake_mode(brakeMode);
    topBackLeftMotor.set_brake_mode(brakeMode);
    bottomBackLeftMotor.set_brake_mode(brakeMode);
    frontRightMotor.set_brake_mode(brakeMode);
    topBackRightMotor.set_brake_mode(brakeMode);
    bottomBackRightMotor.set_brake_mode(brakeMode);
}

// Store an action to be executed during the next movement at the given error
void Drivetrain::addAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn); // construct in place
}

// Converts encoder rotations to inches traveled
long double Drivetrain::ticksToInches(int ticks) {
    return trackingWheelDiameter * ticks * conversions::pi / 360;
}

// Carry out one step of odometry calculations, called in main task
void Drivetrain::trackPosition() {

    using conversions::radians;
    using conversions::degrees;

    // static state variables to calculate change in sensor values
    static int32_t lastParallelValue        = 0;
    static int32_t lastPerpendicularValue   = 0;
    static double lastInertialAngle         = 0;

    /**
     * Main Calculations
     */

    // get encoder values
    int32_t parallelValue       = parallelTrackingWheel.get_value();
    int32_t perpendicularValue  = perpendicularTrackingWheel.get_value();

    // Average imu values, make negative since imu uses clockwise is positive notation,
    // but Drivetrain uses counterclockwise is positive
    double inertialAngle        = -(imu1.get_rotation() + imu2.get_rotation()) / 2.0;

    // get tracking wheel travel distances
    long double parallelDist       = ticksToInches(parallelValue - lastParallelValue);
    long double perpendicularDist  = ticksToInches(perpendicularValue - lastPerpendicularValue);

    // update tracked sensor values for next call to the function
    lastParallelValue       = parallelValue;
    lastPerpendicularValue  = perpendicularValue;

    long double angle = radians(inertialAngle - lastInertialAngle);
    lastInertialAngle = inertialAngle;

    // use tracking wheel travel distances and change in heading to get local displacement
    long double distMain    = angle == 0 ? parallelDist : 2 * (parallelDist / angle + wheelSpacingParallel) * sin(angle / 2);
    long double distSlide   = angle == 0 ? perpendicularDist : 2 * (perpendicularDist / angle + wheelSpacingPerpendicular) * sin(angle / 2);

    // account for the fact that turning happened throughout the arc
    long double theta = radians(heading) + angle / 2;

    // convert from polar to cartesian coordinates and update position data variables
    xPos += distMain * cos(theta) + distSlide * sin(theta);
    yPos += distMain * sin(theta) - distSlide * cos(theta);
    heading += degrees(angle);

    // wrap heading to be on the interval [0, 360)
    if (heading >= 360) {
        heading -= 360;
    } else if (heading < 0) {
        heading += 360;
    }

}