#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "util/equations.hpp"
#include "pros/rtos.h"

using namespace drive;
using namespace conversions;
using namespace equations;

// constructor
Drivetrain::Action::Action(std::function<void()>&& newAction, double atError, bool duringTurn)
    : action {std::move(newAction)}, error {atError}, duringTurn {duringTurn} {}

// Follows the motion profile stored in the Path, uses feedback error correction during the motion
// Error correction uses pure pursuit to follow a designated point ahead on the point rather than the path itself,
// otherwise, if the bot was off the path it would turn directly into the path rather than smoothly rejoining the path
Drivetrain& Drivetrain::operator<<(const Path& path) {

    /* Initialize state data, update PID targets (ignore the profiles as they are only needed for error correction) */
    
    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(path.target.x, path.target.y);
    }

    linearPID.setNewTarget(path.lookAheadDistance, true); // follow the look ahead point at the look ahead distance
    rotPID.setNewTarget(0, true);

    for (const Path::Velocities& velocitySet : path) { // go through the path

        uint32_t startTime = pros::millis();

    positionDataMutex.take(TIMEOUT_MAX);

        /* Error correction calculation */

        // target the look ahead point
        long double curDist = distance(velocitySet.xExtension - xPos, velocitySet.yExtension - yPos);
        long double overallDist = distance(path.target.x - xPos, path.target.y - yPos);

        int linearOutput = abs(linearPID.calcPower(curDist)); // get PIDController output

        double rawAngle = atan2(velocitySet.yExtension - yPos, velocitySet.xExtension - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle) - (driveReversed ? 180 : 0); // face backwards if following in reverse
        if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle); // target the look ahead point
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle)); // get PIDController output
       
    positionDataMutex.give();

        // power the Drivetrain as determined by the motion profile and error correction
        supplyVoltage(
            velocitySet.linearVoltage * (driveReversed ? -1 : 1) + linearOutput * cos(angleToPoint),
            velocitySet.rotVoltage - rotOutput
        );

        // call revavent actions when close enough to the target
        executeActions(overallDist);
        if (stopped) { // end the motion if stopped early
            endMotion(path.target.x, path.target.y);
            linearPID.updatePreviousSystemOutput(
                velocitySet.linearVoltage * (driveReversed ? -1 : 1) + linearOutput * cos(angleToPoint)
            );
            rotPID.updatePreviousSystemOutput(rotOutput - velocitySet.rotVoltage);
            return *this;
        }

        // critical to run this loop as often as possible as allowed by the sensors and motors (100Hz)
        pros::Task::delay_until(&startTime, 10);

    }

    endMotion(path.target.x, path.target.y);
    linearPID.updatePreviousSystemOutput(0);
    rotPID.updatePreviousSystemOutput(0);

    return *this; // allows operator chaining

}

// Uses pure pursuit to move towards the Waypoint; for optimal use utilize several pure pursuit movements in succession
Drivetrain& Drivetrain::operator<<(const Waypoint& p) {

    // reset the static variables in the exit condition function
    p.exitConditions(0, 0, true);

    /* Initialize state data, update PID targets */

    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(p.x, p.y);
    }

    // because we target a distance of 0 from the target, the output of the linear controller will never be positive
    linearPID.setNewTarget(0);
    rotPID.setNewTarget(0);

    while (true) {

    positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

        // target a point (to turn towards) look ahead distance further along on the path
        XYPoint target = purePursuitLookAhead(p.lookAheadDistance, {p.x, p.y});

        double curDist = distance(p.x - xPos, p.y - yPos); // target end of path

        int linearOutput = abs(linearPID.calcPower(curDist)); // get PIDController output

        double rawAngle = atan2(target.y - yPos, target.x - xPos);

        long double angleToPoint = rawAngle - radians(heading);

        long double targetAngle = degrees(rawAngle) - (driveReversed ? 180 : 0); // face backwards if following in reverse
        if (targetAngle < 0) {
            targetAngle += 360;
        }
        rotPID.alterTarget(targetAngle); // target the look ahead point
        int rotOutput = rotPID.calcPower(wrapAngle(targetAngle)); // get PIDController output


    positionDataMutex.give();

        // power the Drivetrain as determined by the PIDControllers and speed limits
        supplyVoltage(
            std::clamp(static_cast<int>(linearOutput * cos(angleToPoint)), -linearSpeedLimit, linearSpeedLimit),
            std::clamp(-rotOutput, -rotSpeedLimit, rotSpeedLimit)
        );

        // call revavent actions when close enough to the target
        executeActions(curDist);

        // end motion if determined by exit conditions or if stopped early
        if (p.exitConditions(curDist, p.lookAheadDistance, false) || stopped) {
            break;
        }

        pros::Task::delay_until(&startTime, 10);

    }

    endMotion(p.x, p.y);

    return *this; // allows operator chaining

}

// Invokes moveTo to move to the Point
Drivetrain& Drivetrain::operator>>(Point p) {
    moveTo(p.x, p.y, p.heading);
    return *this; // allows operator chaining
}

// If heading is a number (is specified), will invoke turnTo after reaching the desired position
void Drivetrain::moveTo(
    long double x, long double y, long double heading,
    LinearExitConditions linearExitConditions, TurnExitConditions turnExitConditions
) {

    // reset the static variables in the exit condition function
    linearExitConditions(0, true, true);

    /* Initialize state data, update PID targets */

    stopped = false;

    if (autoDetermineReversed) {
        determineFollowDirection(x, y);
    }

    bool firstLoop = true; // for quick exit if starting at target

    // because we target a distance of 0 from the target, the output of the linear controller will never be positive
    linearPID.setNewTarget(0);
    rotPID.setNewTarget(targetHeading, true); // ignore the profile for maximal straight line heading correction immediately

    bool canTurn = true;

    while (true) {

    positionDataMutex.take(TIMEOUT_MAX);

        uint32_t startTime = pros::millis();

        double curDist = distance(x - xPos, y - yPos); // target the end point

        int linearOutput = abs(linearPID.calcPower(curDist)); // get PIDController output
        int rotOutput;
        double rawAngle = atan2(y - yPos, x - xPos);
        long double angleToPoint = rawAngle - radians(Drivetrain::heading);

        // if moved into or out of the min distance for turning circle around the target point
        if (canTurn == curDist < minDistForTurning) {
            if (canTurn) { // if moving into the circle, target the current heading
                if (!firstLoop) {
                    targetHeading = Drivetrain::heading;
                }
                if (targetHeading >= 360) {targetHeading -= 360;}
                rotPID.alterTarget(targetHeading);
                canTurn = false;
            } else { // if moving out of the circle, reenable turning
                canTurn = true;
            }
        }
        
        if (canTurn) { // if turning is enabled

            long double targetAngle = degrees(rawAngle) - (driveReversed ? 180 : 0);
            while (targetAngle < 0) {
                targetAngle += 360;
            }
            rotPID.alterTarget(targetAngle); // target the point
            rotOutput = rotPID.calcPower(wrapAngle(targetAngle)); // get PIDController output
        
        } else { // if turning is disabled, target the current heading

            rotOutput = rotPID.calcPower(wrapAngle(targetHeading));
        
        }

    positionDataMutex.give();

        // power the Drivetrain as determined by the PIDControllers and speed limits
        supplyVoltage(
            std::clamp(static_cast<int>(linearOutput * cos(angleToPoint)), -linearSpeedLimit, linearSpeedLimit),
            std::clamp(-rotOutput, -rotSpeedLimit, rotSpeedLimit)
        );

        // call revavent actions when close enough to the target
        executeActions(curDist);

        // end motion if determined by exit conditions or if stopped early
        // ternary operator is used to prevent early exit (of being at the target) during the first loop
        // cos(angleToPoint) ensures the motion can exit when turning is disabled if the bot is slightly to the side of the target
        if (linearExitConditions(curDist * (firstLoop ? 1 : cos(angleToPoint)), firstLoop, false) || stopped) {
            break;
        }
        firstLoop = false;

        pros::Task::delay_until(&startTime, 10);

    }

    if (!isnanf(heading)) { // if a heading was specified, turn to it
        turnTo(
            heading,
            turnExitConditions
        );
        // update old target data (for pure pursuit)
        oldTargetX = x;
        oldTargetY = y;
    } else { // if turnTo was not called (which would call endMotion), end the motion
        endMotion(x, y);
    }

}

// Will turn to face targetForHeading using absolute coordinates after reaching the desired position
void Drivetrain::moveTo(
    long double x, long double y, XYPoint targetForHeading,
    LinearExitConditions linearExitConditions, TurnExitConditions turnExitConditions
) {
    moveTo(x, y, degrees(atan2(targetForHeading.y - y, targetForHeading.x - x)), linearExitConditions, turnExitConditions);
}

// Will move to the desired position
void Drivetrain::moveTo(long double x, long double y, LinearExitConditions linearExitConditions) {
    moveTo(x, y, NAN, linearExitConditions);
}

// Will turn to the desired heading
void Drivetrain::turnTo(long double heading, TurnExitConditions exitConditions) {

    // bound heading to be on the interval [0, 360)
    while (heading >= 360) {
        heading -= 360;
    }
    while (heading < 0) {
        heading += 360;
    }

    // reset the static variables in the exit condition function
    exitConditions(true, true);

    /* Initialize state data, update PID target */

    stopped = false;

    bool firstLoop = true; // for quick exit if starting at target

    targetHeading = heading;

    rotPID.setNewTarget(targetHeading); // target the supplied heading

    while (true) {

    positionDataMutex.take(TIMEOUT_MAX);
        uint32_t startTime = pros::millis();

        int rotOutput = rotPID.calcPower(wrapAngle(targetHeading)); // get PIDController output
    positionDataMutex.give();

        // power the Drivetrain as determined by the PIDController and speed limit
        supplyVoltage(0, std::clamp(-rotOutput, -rotSpeedLimit, rotSpeedLimit));

        // call revavent actions when close enough to the target
        executeActions(fabs(rotPID.getError()), true);

        // end motion if determined by exit conditions or if stopped early
        if (exitConditions(firstLoop, false) || stopped) {
            break;
        }
        firstLoop = false;

        pros::Task::delay_until(&startTime, 10);

    }

    endMotion();

}

// Will turn to face target using the system specified by the bool absolute
void Drivetrain::turnTo(XYPoint target, bool absolute, TurnExitConditions exitConditions) {
    if (absolute) { // use old targets
        turnTo(degrees(atan2(target.y - oldTargetY, target.x - oldTargetX)), exitConditions);
    } else { // use current position
        Point pos = getPosition();
        turnTo(degrees(atan2(target.y - pos.y, target.x - pos.x)), exitConditions);
    }
}

// Moves forward, uses the system specified by the bool absolute to determine the desired final position
// Does not invoke turnTo after the movement is complete
void Drivetrain::moveForward(long double dist, bool absolute, LinearExitConditions exitConditions) {
    if (absolute) { // use old targets
        long double curHeading = radians(targetHeading);
        moveTo(oldTargetX + dist * cos(curHeading), oldTargetY + dist * sin(curHeading), exitConditions);
    } else { // use current position
        Point pos = getPosition();
        long double currHeading = radians(pos.heading);
        moveTo(pos.x + dist * cos(currHeading), pos.y + dist * sin(currHeading), exitConditions);
    }
}

// Updates driveReversed, call if autoDetermineReversed
void Drivetrain::determineFollowDirection(long double xTarget, long double yTarget) {
    double angleToPoint = degrees(atan2(yTarget - oldTargetY, xTarget - oldTargetX));
    if (angleToPoint < 0) {angleToPoint += 360;}
    if (fabs(angleToPoint - wrapTargetHeading(angleToPoint)) > 90) { // if the point is behind the bot
        driveReversed = true;
    } else { // if the point is in front of the bot
        driveReversed = false;
    }
}

// Executes actions stored in actionList if they are eligible to be executed
void Drivetrain::executeActions(double currError, bool inTurn) {

    for (Action& action : actionList) { // check all actions

        if (action.duringTurn == inTurn) { // if the action corresponds to the correct type of motion (move to vs turn in place)

            double& errorToExecute = action.error;

            // if the action has not been executed and the Drivetrain is close enough to the target
            if (errorToExecute != 0 && errorToExecute >= currError) {
                action.action();
                errorToExecute = 0; // mark the action as having been called
            }

        }

    }

}

// Sets motor power to 0, clears actionList
void Drivetrain::endMotion() {
    supply(0, 0);
    actionList.clear();
}

// Sets motor power to 0, updates old target variables, clears actionList
void Drivetrain::endMotion(long double targetX, long double targetY) {
    oldTargetX = targetX; oldTargetY = targetY;
    supply(0, 0);
    actionList.clear();
}

// Wraps the heading based off of targetAngle to minimize the distance between the two
long double Drivetrain::wrapAngle(long double targetAngle) {
    
    long double wrappedAngle;

    // if targetAngle is closer to 0 than 360,
    // account for shortest distance having heading being counterclockwise of targetAngle
    // otherwise account for shortest distance having heading being clockwise of targetAngle
    if (targetAngle < 180) {
        wrappedAngle = (heading - targetAngle >= 180 ? heading - 360 : heading);
    } else {
        wrappedAngle = (targetAngle - heading >= 180 ? heading + 360 : heading);
    }

    return wrappedAngle;

}

// Wraps the targetHeading based off of targetAngle to minimize the distance between the two
long double Drivetrain::wrapTargetHeading(long double targetAngle) {
    
    long double wrappedAngle;

    // If targetAngle is closer to 0 than 360,
    // account for shortest distance having targetHeading being counterclockwise of targetAngle
    // otherwise account for shortest distance having heading being clockwise of targetAngle
    if (targetAngle < 180) {
        wrappedAngle = (targetHeading - targetAngle >= 180 ? targetHeading - 360 : targetHeading);
    } else {
        wrappedAngle = (targetAngle - targetHeading >= 180 ? targetHeading + 360 : targetHeading);
    }

    return wrappedAngle;

}

// Returns 1 if positive or 0, -1 if negative
int Drivetrain::sign(long double num) {
    return (num >= 0 ? 1 : -1);
}

// Returns the point for the movement algorithm to target, as determined by pure pursuit
Drivetrain::XYPoint Drivetrain::purePursuitLookAhead(
    long double lookAheadDistance,
    XYPoint newEndpoint
) {

    /* Get endpoint location relative to the Drivetrain (the Drivetrain is treated as the origin) */

    long double x1 = oldTargetX - xPos;
    long double y1 = oldTargetY - yPos;
    long double x2 = newEndpoint.x - xPos;
    long double y2 = newEndpoint.y - yPos;

    // find change in coordinates between endpoints
    long double dx = x2 - x1;
    long double dy = y2 - y1;

    long double dr = dx * dx + dy * dy;
    if (dr == 0) {return newEndpoint;} // return early if the two endpoints are the same point
    long double d = x1 * y2 - x2 * y1;
    // fabs within the square root to get a real result
    long double discriminant = sqrt(fabs(lookAheadDistance * lookAheadDistance * dr - d * d));

    // find x and y coordinates (two pairs) on the line segment look ahead distance from the Drivetrain
    long double newX1 = (d * dy + sign(dy) * dx * discriminant) / dr;
    long double newY1 = (-d * dx + fabs(dy) * discriminant) / dr;
    long double newX2 = (d * dy - sign(dy) * dx * discriminant) / dr;
    long double newY2 = (-d * dx - fabs(dy) * discriminant) / dr;

    // find which target point is closer
    double dist1 = distance(newX1 - x2, newY1 - y2);
    double dist2 = distance(newX2 - x2, newY2 - y2);

    // return the closer potential target point, convert the coordinates back to field centric
    if (dist1 <= dist2) {
        return {newX1 + xPos, newY1 + yPos};
    }
    return {newX2 + xPos, newY2 + yPos};

}