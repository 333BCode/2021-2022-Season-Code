#include "drivetrain.hpp"

using namespace drive;

Point::Point(long double x, long double y, long double heading)
    : x {x}, y {y}, heading {heading} {}

Point& Point::withAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn);
    return *this;
}

Waypoint::Waypoint(long double x, long double y)
    : x {x}, y {y} {}

Waypoint& Waypoint::withAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actionList.emplace_back(std::move(action), dist, duringTurn);
    return *this;
}

Waypoint& Waypoint::withLookAhead(long double newLookAhead) {
    lookAheadDistance = newLookAhead;
    return *this;
}

Waypoint& Waypoint::withExitCondition(PurePursuitExitConditions newExitConditions) {
    exitConditions = newExitConditions;
    return *this;
}