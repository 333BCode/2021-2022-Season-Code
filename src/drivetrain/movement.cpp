#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "pros/rtos.h"

using namespace drive;
using namespace conversions;

bool Drivetrain::stopped = false;

Drivetrain& Drivetrain::operator<<(const Point& p) {

    stopped = false;

    linearPID.setNewTarget(0);
    if (!isnanf(p.heading) && state == State::enabledStrafing) {
        targetHeading = p.heading;
    }
    rotPID.setNewTarget(targetHeading);

    while (!stopped) {

        positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        XYPoint target = purePursuitLookAhead(p.lookAheadDistance, {p.x, p.y});

        int linearOutput = linearPID.calcPower(distance(p.x - xPos, p.y - yPos));
        int rotOutput;
        long double angleToPoint = atan2(target.y - yPos, target.x - xPos) - radians(heading);

        if (state == State::enabledStrafing) {

            /*
             * Holonomic movement control
             */

            rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
            supplyVoltage(linearOutput * cos(angleToPoint), -linearOutput * sin(angleToPoint), rotOutput);

        } else {

            /*
             * Linear movement control
             */

            long double targetAngle = degrees(atan2(target.y - yPos, target.x - xPos));
            if (targetAngle < 0) {targetAngle += 360;}
            rotPID.alterTarget(targetAngle);
            rotOutput = rotPID.calcPower(wrapAngle(targetAngle));

            supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        }

        executeActions(p);

        positionDataMutex.give();

        if (distance(p.x - xPos, p.y - yPos) < p.lookAheadDistance) {
            stop();
            break;
        }

        pros::Task::delay_until(&startTime, 10);

    }

    oldTargetX = p.x;
    oldTargetY = p.y;

    return *this;

}
Drivetrain& Drivetrain::operator>>(const Point& p) {

    if (oldTargetX == p.x && oldTargetY == p.y) {
        if (!isnanf(p.heading)) {
            turnTo(p.heading, p.exitConditions);
        }
        return *this;
    }

    stopped = false;

    uint16_t count = 0;

    linearPID.setNewTarget(0);
    if (!isnanf(p.heading) && state == State::enabledStrafing) {
        targetHeading = p.heading;
    }
    rotPID.setNewTarget(targetHeading);

    bool canTurn = true;

    while (!stopped) {

        positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        int linearOutput = linearPID.calcPower(distance(p.x - xPos, p.y - yPos));
        int rotOutput;
        long double angleToPoint = atan2(p.y - yPos, p.x - xPos) - radians(heading);

        if (state == State::enabledStrafing) {

            /*
             * Holonomic movement control
             */

            rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
            supplyVoltage(linearOutput * cos(angleToPoint), -linearOutput * sin(angleToPoint), rotOutput);

        } else {

            /*
             * Linear movement control
             */

            if (canTurn && distance(p.x - xPos, p.y - yPos) < minDistForTurning) {
                targetHeading = heading;
                rotPID.alterTarget(targetHeading);
                canTurn = false;
            }
            
            if (canTurn) {

                long double targetAngle = degrees(atan2(p.y - yPos, p.x - xPos));
                if (targetAngle < 0) {targetAngle += 360;}
                rotPID.alterTarget(targetAngle);
                rotOutput = rotPID.calcPower(wrapAngle(targetAngle));
            
            } else {

                rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
            
            }

            supplyVoltage(linearOutput * cos(angleToPoint), rotOutput);

        }

        executeActions(p);

        positionDataMutex.give();

        if (
            fabs(linearPID.getError()) <= p.exitConditions.maxLinearError
            && fabs(linearPID.getDerivative()) <= p.exitConditions.maxLinearDerivative
            && fabs(rotPID.getError()) <= p.exitConditions.maxRotError
            && fabs(rotPID.getDerivative()) <= p.exitConditions.maxRotDerivative
        ) {
            ++count;
            if (count >= p.exitConditions.minTime * 100) {
                stop();
                break;
            }
        } else {
            count = 0;
        }

        pros::Task::delay_until(&startTime, 10);

    }

    oldTargetX = p.x;
    oldTargetY = p.y;

    return *this;

}

void Drivetrain::turnTo(long double heading, ExitConditions exitConditions) {

    targetHeading = heading;

    uint16_t count = 0;

    rotPID.setNewTarget(targetHeading);

    while (true) {

        positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        int rotOutput = rotPID.calcPower(wrapAngle(targetHeading));

        supplyVoltage(0, rotOutput);

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

        pros::Task::delay_until(&startTime, 10);

    }

    stop();

}

Point Drivetrain::forward(long double dist) {
    return {dist * cos(heading), dist * sin(heading)};
}

void Drivetrain::executeActions(const Point& p) {

    double error = distance(p.x - xPos, p.y - yPos);

    for (size_t i = 0; i < p.actions.size(); ++i) {

        double& distance = const_cast<double&>(p.actions[i].distance);

        if (distance != 0 && distance >= error) {
            p.actions[i].action();
            distance = 0;
        }

    }

}

double Drivetrain::distance(double dx, double dy) {
    return sqrt(dx * dx + dy * dy);
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