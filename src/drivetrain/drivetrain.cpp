#include "drivetrain.hpp"
#include "util/conversions.hpp"

namespace drive {

Drivetrain base;

} // namespace drive

pros::Mutex Drivetrain::positionDataMutex {};

long double Drivetrain::xPos    = 72;
long double Drivetrain::yPos    = 72;
long double Drivetrain::heading = 90;

void Drivetrain::operator()(const State& newState) {
    state = newState;
}

Drivetrain::State Drivetrain::getState() {
    return state;
}

std::array<long double, 3> Drivetrain::getPosition() {
    return {xPos, yPos, heading};
}

void Drivetrain::setPosition(long double newX, long double newY, long double newHeading) {
    xPos = newX; yPos = newY; heading = newHeading;
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

long double Drivetrain::ticksToInches(int ticks) {
    return trackingWheelDiameter * ticks * conversions::pi / 360;
}

void Drivetrain::trackPosition() {

    using conversions::radians;
    using conversions::degrees;

    static int32_t lastLeftValue      = 0;
    static int32_t lastRightValue     = 0;
    static int32_t lastMiddleValue    = 0;
    static double lastInertialAngle        = 0;

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