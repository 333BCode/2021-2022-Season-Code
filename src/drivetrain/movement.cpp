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

    linearPID.setNewTarget(path.lookAheadDistance, true);
    rotPID.setNewTarget(0, true);

    uint32_t startTime = pros::millis();

    for (const Path::Velocities& velocitySet : path) {

    positionDataMutex.take(TIMEOUT_MAX);

        long double curDist = distance(velocitySet.xExtension - xPos, velocitySet.yExtension - yPos);
        long double overallDist = distance(path.target.x - xPos, path.target.y - yPos);

        int linearOutput = linearPID.calcPower(curDist);

        double rawAngle = atan2(velocitySet.yExtension - yPos, velocitySet.xExtension - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle);
        if (driveReversed) {
            targetAngle += 180;
        } else if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle);
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
        
    positionDataMutex.give();

        supplyVoltage(
            velocitySet.linearVoltage * (driveReversed ? -1 : 1) + linearOutput * cos(angleToPoint),
            velocitySet.rotVoltage + rotOutput
        );

        executeActions(overallDist);
        if (stopped) {
            break;
        }

        pros::Task::delay_until(&startTime, 10);
        startTime += 10;

    }

    endMotion(path.target.x, path.target.y);

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

        double curDist = distance(p.x - xPos, p.y - yPos);

        int linearOutput = linearPID.calcPower(curDist);

        double rawAngle = atan2(target.y - yPos, target.x - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle);
        if (driveReversed) {
            targetAngle += 180;
        } else if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle);
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle));


    positionDataMutex.give();

        supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        executeActions(curDist);

        if (curDist < p.lookAheadDistance) {
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

    stopped = false;

    bool firstLoop = true;

    uint16_t count = 0;

    linearPID.setNewTarget(0);
    rotPID.setNewTarget(targetHeading);

    bool canTurn = true;

    while (!stopped) {

    positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

        double curDist = distance(x - xPos, y - yPos);

        int linearOutput = linearPID.calcPower(curDist);
        int rotOutput;
        double rawAngle = atan2(y - yPos, x - xPos);
        long double angleToPoint = rawAngle - radians(heading);

        if (canTurn == curDist < minDistForTurning) {
            if (canTurn) {
                targetHeading = heading;
                rotPID.alterTarget(targetHeading);
                canTurn = false;
            } else {
                canTurn = true;
            }
        }
        
        if (canTurn) {

            long double targetAngle = degrees(rawAngle);
            if (driveReversed) {
                targetAngle += 180;
            } else if (targetAngle < 0) {
                targetAngle += 360;
            }
            rotPID.alterTarget(targetAngle);
            rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
        
        } else {

            rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
        
        }

    positionDataMutex.give();

        supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        executeActions(curDist);

        if (firstLoop) {
            if (curDist <= exitConditions.maxLinearError) {
                break;
            } else {
                firstLoop = false;
            }
        } else if (
            curDist * cos(angleToPoint) <= exitConditions.maxLinearError
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

        pros::Task::delay_until(&startTime, 10);

    }

    if (!isnanf(heading)) {
        turnTo(
            driveReversed ? heading + (heading < 180 ? 180 : -180) : heading,
            exitConditions
        );
        oldTargetX = x;
        oldTargetY = y;
    } else {
        endMotion(x, y);
    }

}

void Drivetrain::moveTo(long double x, long double y, const ExitConditions& exitConditions) {
    moveTo(x, y, NAN, exitConditions);
}

void Drivetrain::turnTo(long double heading, const ExitConditions& exitConditions) {

    while (heading >= 360) {
        heading -= 360;
    }
    while (heading < 0) {
        heading += 360;
    }

    stopped = false;

    bool firstLoop = true;

    targetHeading = heading;

    uint16_t count = 0;

    rotPID.setNewTarget(targetHeading);

    while (!stopped) {

    positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        int rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
    positionDataMutex.give();

        supplyVoltage(0, rotOutput);

        executeActions(fabs(rotPID.getError()), true);

        if (firstLoop) {
            if (fabs(rotPID.getError()) <= exitConditions.maxRotError) {
                break;
            } else {
                firstLoop = false;
            }
        } else if (
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
    supply(0, 0);
    actionList.clear();
}

void Drivetrain::endMotion(long double targetX, long double targetY) {
    oldTargetX = targetX; oldTargetY = targetY;
    supply(0, 0);
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