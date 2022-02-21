#include "drivetrain.hpp"

using namespace drive;

Point::Point(long double x, long double y, long double heading)
    : x {x}, y {y}, heading {heading} {}

// Store action
Point& Point::withAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn); // construct in place
    return *this; // allow function chaining
}

Waypoint::Waypoint(long double x, long double y)
    : x {x}, y {y} {}

// Store actions to be executed during the next movement (or pure pursuit segment)
Waypoint& Waypoint::withAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn); // construct in place
    return *this; // allow function chaining
}

// Change the look ahead distance for the movement to this point
Waypoint& Waypoint::withLookAhead(long double newLookAhead) {
    lookAheadDistance = newLookAhead;
    return *this; // allow function chaining
}

// Change the exit conditions for the movement to this point
Waypoint& Waypoint::withExitCondition(PurePursuitExitConditions newExitConditions) {
    exitConditions = newExitConditions;
    return *this; // allow function chaining
}