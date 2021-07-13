#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "util/equations.hpp"

using namespace drive;
using namespace conversions;
using namespace equations;

constexpr size_t defaultAllocCapacity   = 500;
constexpr size_t reallocAddition        = 100;

Path::Path()
    : data {new Velocities[defaultAllocCapacity]}, length {0}, capacity {defaultAllocCapacity} {}

Path::Path(const Path& path)
    : length {path.length}, capacity {path.length}
{
    if (capacity) {

        data = new Velocities[capacity];
        for (size_t i = 0; i < length; ++i) {
            data[i] = path.data[i];
        }

    } else {
        data = nullptr;
    }
}

Path::Path(Path&& path)
    : data {path.data}, length {path.length}, capacity {path.capacity}
{
    if (capacity) {
        path.data = nullptr;
        path.length = 0;
        path.capacity = 0;
    }
}

Path::Path(Velocities* path, size_t length)
    : data {path}, length {length}, capacity {length} {}

Path::Path(Velocities* path, size_t size, size_t capacity)
    : data {path}, length {size}, capacity {capacity} {}

Path::~Path() {
    if (data) {
        delete[] data;
    }
}

void Path::add(long double leftVelocity, long double rightVelocity) {

    if (length == capacity) {

        capacity += reallocAddition;
        Velocities* newData = new Velocities[capacity];

        for (size_t i = 0; i < length; ++i) {
            newData[i] = data[i];
        }

        delete[] data;
        data = newData;

    }

    data[length] = {leftVelocity, rightVelocity};
    ++length;

}

Path::Velocities Path::operator[](size_t index) {
    return data[index];
}

size_t Path::size() {
    return length;
}

Path Path::generatePathTo(XYHPoint point) {
    return generatePath({xPos, yPos, heading}, point);
}

Path Path::generatePathFromOrigin(long double startingHeading, XYHPoint point) {
    return generatePath({0, 0, startingHeading}, point);
}

Path Path::generatePath(XYHPoint start, XYHPoint end) {

    Path profile {};

    long double totalDist = distance(start.x - end.x, start.y - end.y);

    long double vx1 = totalDist * cos(radians(start.heading));
    long double vy1 = totalDist * sin(radians(start.heading));

    long double vx2 = totalDist * cos(radians(end.heading));
    long double vy2 = totalDist * sin(radians(end.heading));

    PolynomialEquation eqx {PATH_POLYNOMIAL_ARGS(start.x, end.y)};
    PolynomialEquation eqxd = eqx.derivative();

    PolynomialEquation eqy {PATH_POLYNOMIAL_ARGS(start.y, end.y)};
    PolynomialEquation eqyd = eqy.derivative();

    CurvatureEquation c = {
        eqxd,
        eqxd.derivative(),
        eqyd,
        eqyd.derivative()
    };

    long double length = (distance(eqxd.at(0), eqyd.at(0)) + distance(eqxd.at(1), eqyd.at(1))) / 2;
    for (long double t = profileDT; t < 1; t += profileDT) {
        length += distance(eqxd.at(t), eqyd.at(t));
    }
    length *= profileDT;

    long double distToAccel = maxVelocity * maxVelocity / (2 * maxAcceleration);
    if (distToAccel > length / 2) {
        distToAccel = length / 2;
    }

    DistanceToTime toTime {eqxd, eqyd, profileDT};

    long double distTraveled = 0;
    while (distTraveled < length) {

        // get the current t for the parametric given the distance traveled
        long double currTime = toTime.atDistance(distTraveled);
        // get the curvature and radius if it exists
        long double curvature = c.at(currTime);
        long double r = (curvature != 0 ? fabs(1 / curvature) : 0);
        // get the maximum forward velocity allowed by the curvature of the path
        long double velocityByCurvature = (curvature == 0 ? maxVelocity : (r * maxVelocity) / (r + drivetrainWidth));
        long double velocityByDistance;

        if (distTraveled < distToAccel) { // if accelerating

            // Velocity given by V^2 = V_0^2 + 2a*deltaX
            // and V = V_0 + a*t (to prevent 0 velocity forever at start
            // of path when distance = 0, *0.5 to get average between endpoints of block of time)
            // this part could use some work
            velocityByDistance = sqrt(2 * maxAcceleration * distTraveled) + 0.5 * maxAcceleration * profileDT;

        } else if (distTraveled > length - distToAccel) { // if deccelerating

            velocityByDistance = sqrt(2 * maxAcceleration * (length - distTraveled));

        } else { // if at cruising speed

            velocityByDistance = maxVelocity;

        }

        // let velocity equal the smaller of the two possible maximum values
        long double velocity = (velocityByCurvature >= velocityByDistance ? velocityByDistance : velocityByCurvature);

        long double rVelocity; long double lVelocity;

        // using largerVelocity / smallerVelocity = (r + DRIVEWIDTH) / (r - DRIVEWIDTH)
        // keep in mind r is for the current position on the path
        if (curvature == 0) {
            // path is straight, r does not exist, both sides have equal speed
            rVelocity = velocity;
            lVelocity = velocity;
        } else if (curvature > 0) {
            // path curves counterclockwise
            rVelocity = ((r + drivetrainWidth) / r) * velocity;
            lVelocity = ((r - drivetrainWidth) / r) * velocity;
        } else {
            // path curves clockwise
            rVelocity = ((r - drivetrainWidth) / r) * velocity;
            lVelocity = ((r + drivetrainWidth) / r) * velocity;
        }

        // add the left and right side velocities to the profile
        profile.add(lVelocity, rVelocity);

        // update the distance traveled
        distTraveled += velocity * profileDT;

    }

    return profile;

}