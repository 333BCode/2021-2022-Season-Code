#include "drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "util/conversions.hpp"
#include "macros.h"

namespace drive {

    Drivetrain base {};

} // namespace drive

bool Drivetrain::calibrated     = false;
bool Drivetrain::driveReversed  = false;

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

void Drivetrain::setReversed(bool reversed) {
    driveReversed = reversed;
}

Drivetrain::Point Drivetrain::getPosition() {
positionDataMutex.take(TIMEOUT_MAX);
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
    backLeftMotor.move(linearPow + rotPow);
    frontRightMotor.move(linearPow - rotPow);
    backRightMotor.move(linearPow - rotPow);
}

void Drivetrain::supplyVoltage(int linearPow, int rotPow) {
    frontLeftMotor.move_voltage(linearPow + rotPow);
    backLeftMotor.move_voltage(linearPow + rotPow);
    frontRightMotor.move_voltage(linearPow - rotPow);
    backRightMotor.move_voltage(linearPow - rotPow);
}

void Drivetrain::supplyVoltagePerSide(int leftVoltage, int rightVoltage) {
    frontLeftMotor.move_voltage(leftVoltage);
    backLeftMotor.move_voltage(leftVoltage);
    frontRightMotor.move_voltage(rightVoltage);
    backRightMotor.move_voltage(rightVoltage);
}

void Drivetrain::stopMotion() {
    stopped = true;
}

void Drivetrain::setBrakeMode(const pros::motor_brake_mode_e_t brakeMode) {
    frontLeftMotor.set_brake_mode(brakeMode);
    backLeftMotor.set_brake_mode(brakeMode);
    frontRightMotor.set_brake_mode(brakeMode);
    backRightMotor.set_brake_mode(brakeMode);
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

    static int32_t lastRightValue       = 0;
    static int32_t lastMiddleValue      = 0;
    static double lastInertialAngle     = 0;
    static int count = 0;

    /**
     * Main Calculations
     */

    int32_t rightEncoderValue   = rightEncoder.get_value();
    int32_t middleEncoderValue  = middleEncoder.get_value();

    double inertialAngle        = -inertial.get_rotation();

    long double rightDist   = ticksToInches(rightEncoderValue - lastRightValue);
    long double middleDist  = ticksToInches(middleEncoderValue - lastMiddleValue);

    lastRightValue  = rightEncoderValue;
    lastMiddleValue = middleEncoderValue;

    long double angle = radians(inertialAngle - lastInertialAngle);
    lastInertialAngle = inertialAngle;
/*
    long double angle = (rightDist - leftDist) / (2 * wheelSpacingParallel);

    if (!isnanf(inertialAngle)) {
        angle = (angle + radians(inertialAngle - lastInertialAngle)) / 2;
        lastInertialAngle = inertialAngle;
    }
*/
    long double distMain    = angle == 0 ? rightDist : 2 * (rightDist / angle + wheelSpacingParallel) * sin(angle / 2);
    long double distSlide   = angle == 0 ? middleDist : 2 * (middleDist / angle + wheelSpacingPerpendicular) * sin(angle / 2);

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