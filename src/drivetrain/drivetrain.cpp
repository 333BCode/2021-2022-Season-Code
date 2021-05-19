#include "drivetrain.hpp"
#include "pros/rtos.h"
#include "util/conversions.hpp"

namespace drive {

    Drivetrain base;

    Drivetrain::Point (*const forward)(long double) = Drivetrain::forward;

} // namespace drive

bool Drivetrain::calibrated = false;

pros::Mutex Drivetrain::positionDataMutex {};
pros::Mutex Drivetrain::calibrationMutex {};

long double Drivetrain::xPos    = 72;
long double Drivetrain::yPos    = 72;
long double Drivetrain::heading = 90;

long double Drivetrain::oldTargetX      = 72;
long double Drivetrain::oldTargetY      = 72;
long double Drivetrain::targetHeading   = 90;

bool Drivetrain::isCalibrated() {
    calibrationMutex.take(TIMEOUT_MAX);
    bool isCalibrated = calibrated;
    calibrationMutex.give();
    return isCalibrated;
}

void Drivetrain::operator()(const State& newState) {
    state = newState;
}

Drivetrain::State Drivetrain::getState() {
    return state;
}

Drivetrain::XYHPoint Drivetrain::getPosition() {
    return {xPos, yPos, heading};
}

void Drivetrain::setPosition(long double newX, long double newY, long double newHeading) {
    xPos = newX; yPos = newY; heading = newHeading;
    oldTargetX = newX; oldTargetY = newY; targetHeading = newHeading;
}

void Drivetrain::supply(int linearPow, int rotPow) {
    frontLeftMotor.move(linearPow + rotPow);
    backLeftMotor.move(linearPow + rotPow);
    frontRightMotor.move(linearPow - rotPow);
    backRightMotor.move(linearPow - rotPow);
}

void Drivetrain::supply(int linearPow, int strafePow, int rotPow) {

    if (state == State::enabledStrafing) {

        frontLeftMotor.move(linearPow + strafePow + rotPow);
        backLeftMotor.move(linearPow - strafePow + rotPow);
        frontRightMotor.move(linearPow - strafePow - rotPow);
        backRightMotor.move(linearPow + strafePow - rotPow);

    } else {

        supply(linearPow, rotPow);

    }

}

void Drivetrain::supplyVoltage(int linearPow, int rotPow) {
    frontLeftMotor.move_voltage(linearPow + rotPow);
    backLeftMotor.move_voltage(linearPow + rotPow);
    frontRightMotor.move_voltage(linearPow - rotPow);
    backRightMotor.move_voltage(linearPow - rotPow);
}

void Drivetrain::supplyVoltage(int linearPow, int strafePow, int rotPow) {

    if (state == State::enabledStrafing) {

        frontLeftMotor.move_voltage(linearPow + strafePow + rotPow);
        backLeftMotor.move_voltage(linearPow - strafePow + rotPow);
        frontRightMotor.move_voltage(linearPow - strafePow - rotPow);
        backRightMotor.move_voltage(linearPow + strafePow - rotPow);

    } else {

        supplyVoltage(linearPow, rotPow);

    }

}

void Drivetrain::stop(const pros::motor_brake_mode_e_t brakeMode) {
    frontLeftMotor.move(0); frontLeftMotor.set_brake_mode(brakeMode);
    backLeftMotor.move(0); backLeftMotor.set_brake_mode(brakeMode);
    frontRightMotor.move(0); frontRightMotor.set_brake_mode(brakeMode);
    backRightMotor.move(0); backRightMotor.set_brake_mode(brakeMode);
    stopped = true;
}

long double Drivetrain::ticksToInches(int ticks) {
    return trackingWheelDiameter * ticks * conversions::pi / 360;
}

void Drivetrain::trackPosition() {

    using conversions::radians;
    using conversions::degrees;

    static int32_t lastLeftValue        = 0;
    static int32_t lastRightValue       = 0;
    static int32_t lastMiddleValue      = 0;
    static double lastInertialAngle     = 0;

    /**
     * Main Calculations
     */

    int32_t leftEncoderValue   = leftEncoder.get_value();
    int32_t rightEncoderValue  = rightEncoder.get_value();
    int32_t middleEncoderValue = middleEncoder.get_value();
    double inertialAngle            = -inertial.get_rotation();

    long double leftDist    = ticksToInches(leftEncoderValue - lastLeftValue);
    long double rightDist   = ticksToInches(rightEncoderValue - lastRightValue);
    long double middleDist  = ticksToInches(middleEncoderValue - lastMiddleValue);

    lastLeftValue   = leftEncoderValue;
    lastRightValue  = rightEncoderValue;
    lastMiddleValue = middleEncoderValue;

    long double angle = (rightDist - leftDist) / (2 * wheelSpacingParallel);
    if (!isnanf(inertialAngle)) {
        angle = (angle + radians(inertialAngle - lastInertialAngle)) / 2;
        lastInertialAngle = inertialAngle;
    }

    long double distMain    = angle == 0 ? leftDist : 2 * (leftDist / angle + wheelSpacingParallel) * sin(angle / 2);
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