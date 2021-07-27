#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "util/equations.hpp"
#include "pros/rtos.h"

using namespace drive;
using namespace conversions;
using namespace equations;

bool Drivetrain::stopped = false;

Drivetrain::Action::Action(std::function<void()>&& newAction, double atError, bool duringTurn)
    : action {std::move(newAction)}, error {atError}, duringTurn {duringTurn} {}

Drivetrain& Drivetrain::operator<<(const Path& path) {

    stopped = false;

    const long double kV = 12000 / maxVelocity;

    uint32_t startTime = pros::millis();

    for (const Path::Velocities& velocitySet : path) {
        supplyVoltagePerSide(velocitySet.leftVelocity * kV, velocitySet.rightVelocity * kV);

        executeActions(path.totalDist - velocitySet.distanceAlongPath);
        if (stopped) {
            break;
        }

        pros::Task::delay_until(&startTime, 10);
        startTime += 10;
    }

    endMotion(path.targetX, path.targetY);

    return *this;

}

Drivetrain& Drivetrain::operator<<(const Waypoint& p) {

    stopped = false;

    linearPID.setNewTarget(0);
    rotPID.setNewTarget(0);

    while (!stopped) {

        positionDataMutex.take(TIMEOUT_MAX);

            uint32_t startTime = pros::millis();

            XYPoint target = purePursuitLookAhead(p.lookAheadDistance, {p.x, p.y});

            int linearOutput = linearPID.calcPower(distance(p.x - xPos, p.y - yPos));
            int rotOutput;
            long double angleToPoint = atan2(target.y - yPos, target.x - xPos) - radians(heading);

            long double targetAngle = degrees(atan2(target.y - yPos, target.x - xPos));
            if (targetAngle < 0) {targetAngle += 360;}
            rotPID.alterTarget(targetAngle);
            rotOutput = rotPID.calcPower(wrapAngle(targetAngle));

        positionDataMutex.give();

        supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        executeActions(fabs(linearPID.getError()));

        Point position = getPosition();
        if (distance(p.x - position.x, p.y - position.y) < p.lookAheadDistance) {
            break;
        }

        pros::Task::delay_until(&startTime, 10);

    }

    endMotion(p.x, p.y);

    return *this;

}

Drivetrain& Drivetrain::operator>>(Point p) {
    moveTo(p.x, p.y, p.heading);
    return *this;
}

void Drivetrain::moveTo(
    long double x, long double y, long double heading, const ExitConditions& exitConditions
) {

    positionDataMutex.take(TIMEOUT_MAX);
        if (distance(x - xPos, y - yPos) <= exitConditions.maxLinearError) {
            if (!isnanf(heading)) {
                turnTo(heading, exitConditions);
            }
        }
    positionDataMutex.give();

    stopped = false;

    uint16_t count = 0;

    linearPID.setNewTarget(0);
    rotPID.setNewTarget(targetHeading);

    bool canTurn = true;

    while (!stopped) {

        positionDataMutex.take(TIMEOUT_MAX);

            uint32_t startTime = pros::millis();

            int linearOutput = linearPID.calcPower(distance(x - xPos, y - yPos));
            int rotOutput;
            long double angleToPoint = atan2(y - yPos, x - xPos) - radians(heading);

            if (canTurn && distance(x - xPos, y - yPos) < minDistForTurning) {
                targetHeading = heading;
                rotPID.alterTarget(targetHeading);
                canTurn = false;
            }
            
            if (canTurn) {

                long double targetAngle = degrees(atan2(y - yPos, x - xPos));
                if (targetAngle < 0) {targetAngle += 360;}
                rotPID.alterTarget(targetAngle);
                rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
            
            } else {

                rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
            
            }

        positionDataMutex.give();

        if (
            fabs(linearPID.getError()) <= exitConditions.maxLinearError
            && fabs(linearPID.getDerivative()) <= exitConditions.maxLinearDerivative
            && fabs(rotPID.getError()) <= exitConditions.maxRotError
            && fabs(rotPID.getDerivative()) <= exitConditions.maxRotDerivative
        ) {
            ++count;
            if (count >= exitConditions.minTime * 100) {
                break;
            }
        } else {
            count = 0;
        }

        supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        executeActions(fabs(linearPID.getError()));

        pros::Task::delay_until(&startTime, 10);

    }

    if (!isnanf(heading)) {
        turnTo(heading, exitConditions);
    }

    endMotion(x, y);

}

void Drivetrain::moveTo(long double x, long double y, const ExitConditions& exitConditions) {
    moveTo(x, y, NAN, exitConditions);
}

void Drivetrain::turnTo(long double heading, const ExitConditions& exitConditions) {

    stopped = false;

    targetHeading = heading;

    uint16_t count = 0;

    rotPID.setNewTarget(targetHeading);

    while (!stopped) {

        positionDataMutex.take(TIMEOUT_MAX);
            uint32_t startTime = pros::millis();

            int rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
        positionDataMutex.give();

        if (
            fabs(rotPID.getError()) <= exitConditions.maxRotError
            && fabs(rotPID.getDerivative()) <= exitConditions.maxRotDerivative
        ) {
            ++count;
            if (count >= exitConditions.minTime * 100) {
                break;
            }
        } else {
            count = 0;
        }

        supplyVoltage(0, rotOutput);

        executeActions(fabs(rotPID.getError()), true);

        pros::Task::delay_until(&startTime, 10);

    }

    endMotion();

}

void Drivetrain::moveForward(long double dist) {
    long double currHeading = getPosition().heading;
    moveTo(dist * cos(currHeading), dist * sin(currHeading), currHeading);
}

void Drivetrain::executeActions(double currError, bool inTurn) {

    for (Action& action : actionList) {

        if (action.duringTurn == inTurn) {

            double& errorToExecute = action.error;

            if (errorToExecute != 0 && errorToExecute >= currError) {
                action.action();
                errorToExecute = 0;
            }

        }

    }

}

void Drivetrain::endMotion() {
    base.supply(0, 0);
    actionList.clear();
}

void Drivetrain::endMotion(long double targetX, long double targetY) {
    oldTargetX = targetX; oldTargetY = targetY;
    base.supply(0, 0);
    actionList.clear();
}

long double Drivetrain::wrapAngle(long double targetAngle) {
    
    long double wrappedAngle;

    if (targetAngle < 180) {
        wrappedAngle = (heading - targetAngle >= 180 ? heading - 360 : heading);
    } else {
        wrappedAngle = (targetAngle - heading >= 180 ? heading + 360 : heading);
    }

    return wrappedAngle;

}

int sign(long double num) {
    return (num >= 0 ? 1 : -1);
}

Drivetrain::XYPoint Drivetrain::purePursuitLookAhead(
    long double lookAheadDistance,
    XYPoint newEndpoint
) {

    long double x1 = oldTargetX - xPos;
    long double y1 = oldTargetY - yPos;
    long double x2 = newEndpoint.x - xPos;
    long double y2 = newEndpoint.y - yPos;

    long double dx = x2 - x1;
    long double dy = y2 - y1;

    long double dr = dx * dx + dy * dy;
    if (dr == 0) {return newEndpoint;}
    long double d = x1 * y2 - x2 * y1;
    long double discriminant = sqrt(fabs(lookAheadDistance * lookAheadDistance * dr - d * d));

    long double newX1 = (d * dy + sign(dy) * dx * discriminant) / dr;
    long double newY1 = (-d * dx + fabs(dy) * discriminant) / dr;
    long double newX2 = (d * dy - sign(dy) * dx * discriminant) / dr;
    long double newY2 = (-d * dx - fabs(dy) * discriminant) / dr;

    double dist1 = distance(newX1 - x2, newY1 - y2);
    double dist2 = distance(newX2 - x2, newY2 - y2);

    if (dist1 <= dist2) {
        return {newX1 + xPos, newY1 + yPos};
    }
    return {newX2 + xPos, newY2 + yPos};

}