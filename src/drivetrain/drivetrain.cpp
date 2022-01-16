#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "util/conversions.hpp"
#include "macros.h"

namespace drive {

    Drivetrain base {};

} // namespace drive

bool Drivetrain::calibrated             = false;
bool Drivetrain::autoDetermineReversed  = false;
bool Drivetrain::driveReversed          = false;

bool Drivetrain::stopped = false;

int Drivetrain::maxSpeed = 12000;

pros::Mutex Drivetrain::positionDataMutex {};

long double Drivetrain::xPos    = 72;
long double Drivetrain::yPos    = 72;
long double Drivetrain::heading = 90;

long double Drivetrain::oldTargetX      = 72;
long double Drivetrain::oldTargetY      = 72;
long double Drivetrain::targetHeading   = 90;

std::vector<Drivetrain::Action> Drivetrain::actionList {};

void Drivetrain::waitUntilCalibrated() {
    while (true) {
    positionDataMutex.take(TIMEOUT_MAX);
        bool isCalibrated = calibrated;
    positionDataMutex.give();
        if (isCalibrated) {
            break;
        }
        pros::delay(10);
    }
}

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

void Drivetrain::setMaxSpeed(int speed) {
    maxSpeed = speed;
}

Drivetrain::Point Drivetrain::getPosition() {
positionDataMutex.take(20);
    Point position = {xPos, yPos, heading};
positionDataMutex.give();
    return position;
}

void Drivetrain::setPosition(long double newX, long double newY, long double newHeading) {
positionDataMutex.take(TIMEOUT_MAX);
    xPos = newX; yPos = newY;
    if (newHeading >= 360) {
        heading = newHeading - 360;
    } else if (newHeading < 0) {
        heading = newHeading + 360;
    } else {
        heading = newHeading;
    }
    oldTargetX = newX; oldTargetY = newY; targetHeading = heading;
positionDataMutex.give();
}

void Drivetrain::supply(int linearPow, int rotPow) {
    frontLeftMotor.move(linearPow + rotPow);
    topBackLeftMotor.move(linearPow + rotPow);
    bottomBackLeftMotor.move(linearPow + rotPow);
    frontRightMotor.move(linearPow - rotPow);
    topBackRightMotor.move(linearPow - rotPow);
    bottomBackRightMotor.move(linearPow - rotPow);
}

void Drivetrain::supplyVoltage(int linearPow, int rotPow) {
    frontLeftMotor.move_voltage(linearPow + rotPow);
    topBackLeftMotor.move_voltage(linearPow + rotPow);
    bottomBackLeftMotor.move_voltage(linearPow + rotPow);
    frontRightMotor.move_voltage(linearPow - rotPow);
    topBackRightMotor.move_voltage(linearPow - rotPow);
    bottomBackRightMotor.move_voltage(linearPow - rotPow);
}

void Drivetrain::stopMotion() {
    stopped = true;
}

void Drivetrain::setBrakeMode(const pros::motor_brake_mode_e_t brakeMode) {
    frontLeftMotor.set_brake_mode(brakeMode);
    topBackLeftMotor.set_brake_mode(brakeMode);
    bottomBackLeftMotor.set_brake_mode(brakeMode);
    frontRightMotor.set_brake_mode(brakeMode);
    topBackRightMotor.set_brake_mode(brakeMode);
    bottomBackRightMotor.set_brake_mode(brakeMode);
}

void Drivetrain::addAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn);
}

long double Drivetrain::ticksToInches(int ticks) {
    return trackingWheelDiameter * ticks * conversions::pi / 360;
}

void Drivetrain::trackPosition() {

    using conversions::radians;
    using conversions::degrees;

    static int32_t lastParallelValue        = 0;
    static int32_t lastPerpendicularValue   = 0;
    static double lastInertialAngle         = 0;
    static int count = 0;

    /**
     * Main Calculations
     */

    int32_t parallelValue       = parallelTrackingWheel.get_position();
    int32_t perpendicularValue  = perpendicularTrackingWheel.get_value();

    double inertialAngle        = -(imu1.get_rotation() + imu2.get_rotation()) / 2.0;

    long double parallelDist       = ticksToInches(parallelValue - lastParallelValue) / 100.0;
    long double perpendicularDist  = ticksToInches(perpendicularValue - lastPerpendicularValue);

    lastParallelValue       = parallelValue;
    lastPerpendicularValue  = perpendicularValue;

    long double angle = radians(inertialAngle - lastInertialAngle);
    lastInertialAngle = inertialAngle;
/*
    long double angle = (rightDist - leftDist) / (2 * wheelSpacingParallel);

    if (!isnanf(inertialAngle)) {
        angle = (angle + radians(inertialAngle - lastInertialAngle)) / 2;
        lastInertialAngle = inertialAngle;
    }
*/
    long double distMain    = angle == 0 ? parallelDist : 2 * (parallelDist / angle + wheelSpacingParallel) * sin(angle / 2);
    long double distSlide   = angle == 0 ? perpendicularDist : 2 * (perpendicularDist / angle + wheelSpacingPerpendicular) * sin(angle / 2);

    long double theta = radians(heading) + angle / 2;

    xPos += distMain * cos(theta) + distSlide * sin(theta);
    yPos += distMain * sin(theta) - distSlide * cos(theta);
    heading += degrees(angle);

    if (heading >= 360) {
        heading -= 360;
    } else if (heading < 0) {
        heading += 360;
    }

}