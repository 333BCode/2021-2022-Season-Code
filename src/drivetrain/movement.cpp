#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "util/equations.hpp"
#include "pros/rtos.h"

using namespace drive;
using namespace conversions;
using namespace equations;

Drivetrain::Action::Action(std::function<void()>&& newAction, double atError, bool duringTurn)
    : action {std::move(newAction)}, error {atError}, duringTurn {duringTurn} {}

Drivetrain& Drivetrain::operator<<(const Path& path) {

    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(path.target.x, path.target.y);
    }

    linearPID.setNewTarget(path.lookAheadDistance, true);
    rotPID.setNewTarget(0, true);

    uint32_t startTime = pros::millis();

    for (const Path::Velocities& velocitySet : path) {

    positionDataMutex.take(TIMEOUT_MAX);

        long double curDist = distance(velocitySet.xExtension - xPos, velocitySet.yExtension - yPos);
        long double overallDist = distance(path.target.x - xPos, path.target.y - yPos);

        int linearOutput = abs(linearPID.calcPower(curDist));

        double rawAngle = atan2(velocitySet.yExtension - yPos, velocitySet.xExtension - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle);
        if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle);
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
        
    positionDataMutex.give();

        supplyVoltage(
            velocitySet.linearVoltage * (driveReversed ? -1 : 1) + linearOutput * cos(angleToPoint),
            velocitySet.rotVoltage - rotOutput
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

    p.exitConditions(0, 0, false);

    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(p.x, p.y);
    }

    linearPID.setNewTarget(0);
    rotPID.setNewTarget(0);

    while (!stopped) {

    positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

        XYPoint target = purePursuitLookAhead(p.lookAheadDistance, {p.x, p.y});

        double curDist = distance(p.x - xPos, p.y - yPos);

        int linearOutput = abs(linearPID.calcPower(curDist));

        double rawAngle = atan2(target.y - yPos, target.x - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle) - (driveReversed ? 180 : 0);
        if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle);
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle));


    positionDataMutex.give();

        supplyVoltage(std::clamp(linearOutput * cos(angleToPoint), -speedLimit, speedLimit), -rotOutput);

        executeActions(curDist);

        if (p.exitConditions(curDist, p.lookAheadDistance, false)) {
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
    long double x, long double y, long double heading,
    LinearExitConditions linearExitConditions, TurnExitConditions turnExitConditions
) {

    linearExitConditions(0, true, true);

    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(x, y);
    }

    bool firstLoop = true;

    uint16_t count = 0;

    linearPID.setNewTarget(0);
    rotPID.setNewTarget(targetHeading);

    bool canTurn = true;

    while (!stopped) {

    positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

        double curDist = distance(x - xPos, y - yPos);

        int linearOutput = abs(linearPID.calcPower(curDist));
        int rotOutput;
        double rawAngle = atan2(y - yPos, x - xPos);
        long double angleToPoint = rawAngle - radians(Drivetrain::heading);

        if (canTurn == curDist < minDistForTurning) {
            if (canTurn) {
                targetHeading = Drivetrain::heading;
                if (targetHeading >= 360) {targetHeading -= 360;}
                rotPID.alterTarget(targetHeading);
                canTurn = false;
            } else {
                canTurn = true;
            }
        }
        
        if (canTurn) {

            long double targetAngle = degrees(rawAngle) - (driveReversed ? 180 : 0);
            while (targetAngle < 0) {
                targetAngle += 360;
            }
            rotPID.alterTarget(targetAngle);
            rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
        
        } else {

            rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
        
        }

    positionDataMutex.give();

        supplyVoltage(std::clamp(linearOutput * cos(angleToPoint), -speedLimit, speedLimit), -rotOutput);

        executeActions(curDist);

        if (linearExitConditions(curDist * (firstLoop ? 1 : cos(angleToPoint)), firstLoop, false)) {
            break;
        }
        firstLoop = false;

        pros::Task::delay_until(&startTime, 10);

    }

    if (!isnanf(heading)) {
        turnTo(
            heading,
            turnExitConditions
        );
        oldTargetX = x;
        oldTargetY = y;
    } else {
        endMotion(x, y);
    }

}

void Drivetrain::moveTo(
    long double x, long double y, XYPoint targetForHeading,
    LinearExitConditions linearExitConditions, TurnExitConditions turnExitConditions
) {
    moveTo(x, y, degrees(atan2(targetForHeading.y - y, targetForHeading.x - x)), linearExitConditions, turnExitConditions);
}

void Drivetrain::moveTo(long double x, long double y, LinearExitConditions linearExitConditions) {
    moveTo(x, y, NAN, linearExitConditions);
}

void Drivetrain::turnTo(long double heading, TurnExitConditions exitConditions) {

    while (heading >= 360) {
        heading -= 360;
    }
    while (heading < 0) {
        heading += 360;
    }

    exitConditions(true, true);

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

        supplyVoltage(0, -rotOutput);

        executeActions(fabs(rotPID.getError()), true);

        if (exitConditions(firstLoop, false)) {
            break;
        }
        firstLoop = false;

        pros::Task::delay_until(&startTime, 10);

    }

    endMotion();

}

void Drivetrain::turnTo(XYPoint target, bool absolute, TurnExitConditions exitConditions) {
    if (absolute) {
        turnTo(degrees(atan2(target.y - oldTargetY, target.x - oldTargetX)), exitConditions);
    } else {
        Point pos = getPosition();
        turnTo(degrees(atan2(target.y - pos.y, target.x - pos.x)), exitConditions);
    }
}

void Drivetrain::moveForward(long double dist, bool absolute, LinearExitConditions exitConditions) {
    if (absolute) {
        long double curHeading = radians(heading);
        moveTo(oldTargetX + dist * cos(curHeading), oldTargetY + dist * sin(curHeading), exitConditions);
    } else {
        Point pos = getPosition();
        long double currHeading = radians(pos.heading);
        moveTo(pos.x + dist * cos(currHeading), pos.y + dist * sin(currHeading), exitConditions);
    }
}

void Drivetrain::determineFollowDirection(long double xTarget, long double yTarget) {
positionDataMutex.take(TIMEOUT_MAX);
    double angleToPoint = degrees(atan2(yTarget - oldTargetY, xTarget - oldTargetX));
    if (angleToPoint < 0) {angleToPoint += 360;}
    std::cout << angleToPoint << '\n';
    if (fabs(angleToPoint - wrapAngle(angleToPoint)) > 90) {
        driveReversed = true;
    } else {
        driveReversed = false;
    }
    std::cout << fabs(angleToPoint - wrapAngle(angleToPoint)) << '\n';
positionDataMutex.give();
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

int Drivetrain::sign(long double num) {
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