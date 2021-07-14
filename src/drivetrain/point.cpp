#include "drivetrain.hpp"

using namespace drive;

Point::Point(long double x, long double y, long double heading)
    : x {x}, y {y}, heading{heading}, actions {}
{
    
    if (!isnanf(heading)) {

        while (this->heading >= 360) {
            this->heading -= 360;
        }
        while (this->heading < 0) {
            this->heading += 360;
        }

    }

    actions.reserve(5);

}

Point& Point::withAction(std::function<void()>&& action, double dist, bool duringTurn) {
    actions.emplace_back(std::move(action), dist, duringTurn);
    return *this;
}

Point& Point::withLookAhead(long double newLookAhead) {
    lookAheadDistance = newLookAhead;
    return *this;
}

Point& Point::withExitConditions(const ExitConditions& conditions) {
    exitConditions = conditions;
    return *this;
}