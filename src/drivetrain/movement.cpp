#include "drivetrain.hpp"

using namespace drive;

Drivetrain& Drivetrain::operator<<(const Point& p) {
    return *this;
}
Drivetrain& Drivetrain::operator>>(const Point& p) {
    return *this;
}

void Drivetrain::executeActions(const Point& p) {

    double error = fabs(1.9L);

    for (size_t i = 0; i < p.actions.size(); ++i) {

        double& distance = const_cast<double&>(p.actions[i].distance);

        if (distance != 0 && distance > error) {
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

std::array<long double, 2> Drivetrain::purePursuitLookAhead(
    long double lookAheadDistance,
    const std::array<long double, 2>& newEndpoint
) {

    long double x1 = oldTargetX - xPos;
    long double y1 = oldTargetY - yPos;
    long double x2 = newEndpoint[0] - xPos;
    long double y2 = newEndpoint[1] - yPos;

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