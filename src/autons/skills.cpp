#include "autonomous.hpp"

bool goalRushExitConditions(long double dist, bool firstLoop, bool reset) {

    static int count = 0;
    Point pos = base.getPosition();
    ++count;
    if (count > 250) {
        return true;
    }
    return pos.y >= 60 || dist < 0.5;

}

void skills() {

    base.setPosition(0, 0, 90);

    base.turnTo(0);

    intake.intake();

}