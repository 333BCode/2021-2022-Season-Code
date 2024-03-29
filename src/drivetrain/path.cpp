#include "drivetrain.hpp"
#include "util/conversions.hpp"
#include "util/equations.hpp"

// macro to more easily initialize equations::PolynomialEquation
#define PATH_POLYNOMIAL_ARGS(start, end, v1, v2)        \
    start * -6 + v1 * -3 + end * 6 + v2 * -3, 5,        \
    start * 15 + v1 * 8 + end * -15 + v2 * 7, 4,        \
    start * -10 + v1 * -6 + end * 10 + v2 * -4, 3,      \
    v1, 1,                                              \
    start, 0

using namespace drive;
using namespace conversions;
using namespace equations;

namespace drive {

    Path (*const generatePathTo)(Point)         = Path::generatePathTo;
    Path (*const generatePath)(Point, Point)    = Path::generatePath;

} // namespace drive

// array allocation determinants
constexpr size_t defaultAllocCapacity   = 500;
constexpr size_t reallocAddition        = 100;

// Initialize a Path, called by motion profile generating functions, allocates internal array
Path::Path(Point target, long double lookAheadDist)
    : target {target},
    data {new Velocities[defaultAllocCapacity]}, length {0}, capacity {defaultAllocCapacity},
    lookAheadDistance {lookAheadDist} {}

// copy constructor
Path::Path(const Path& path)
    : target {path.target},
    length {path.length}, capacity {path.length}, lookAheadDistance {path.lookAheadDistance}
{
    if (capacity > 0) {

        // copy over data
        data = new Velocities[length];
        for (size_t i = 0; i < length; ++i) {
            data[i] = path.data[i];
        }

    } else {
        data = nullptr; // no data to copy
    }
}

// move constructor
Path::Path(Path&& path)
    : target {path.target},
    data {path.data}, length {path.length}, capacity {path.capacity}, lookAheadDistance {path.lookAheadDistance}
{
    if (capacity > 0) { // remove path access to array if array transfer occured
        path.data = nullptr;
        path.length = 0;
        path.capacity = 0;
    }
}

// constructor to initialize a Path from a pre-generated array
// This is for profiles generated by external programs
Path::Path(Velocities* path, size_t length, Point target, long double lookAheadDist)
    : target {target},
    data {path}, length {length}, capacity {length}, lookAheadDistance {lookAheadDist} {}

// destructor
Path::~Path() {
    if (capacity > 0) { // free memory if allocated
        delete[] data;
    }
}

// Store an action to be executed during the next movement
Path& Path::withAction(std::function<void()>&& action, double dist) {
    actionList.emplace_back(std::move(action), dist, false); // construct in place
    return *this; // allow function chaining
}

// Adds a new Velocities to the internal array, reallocates memory if needed
void Path::add(int linearVoltage, int rotVoltage, long double xExtension, long double yExtension) {

    if (length == capacity) { // reallocate memory if out of capacity

        capacity += reallocAddition;
        Velocities* oldData = data;
        data = new Velocities[capacity];

        for (size_t i = 0; i < length; ++i) { // copy over data to new array
            data[i] = oldData[i];
        }

        delete[] oldData; // free memory used for the old array

    }

    // update the next element to store a relavent Velocities struct
    data[length] = {linearVoltage, rotVoltage, xExtension, yExtension};
    ++length;

}

// Index the internal array
const Path::Velocities& Path::operator[](size_t index) const {
    return data[index];
}

/* Get iterators, implements for each loop functionality */

const Path::Velocities* Path::begin() const {
    return data;
}

const Path::Velocities* Path::end() const {
    return data + length;
}

// Get the size (not capacity) of the internal array
size_t Path::size() const {
    return length;
}

/**
 * Motion profile generation functions
 * Those without a lookAheadDist parameter are pointed to by function pointers in the drive namespace for easier calls
 */

// start from current tracked position
Path Path::generatePathTo(Point point) {
    return generatePath(getPosition(), point, defaultLookAheadDistance);
}

Path Path::generatePath(Point start, Point end) {
    return generatePath(start, end, defaultLookAheadDistance);
}

// start from current tracked position
Path Path::generatePathTo(Point point, long double lookAheadDist) {
    return generatePath(getPosition(), point, lookAheadDist);
}

// use path and trajectory generation to create a motion profile
Path Path::generatePath(Point start, Point end, long double lookAheadDist) {

    /* initialize specific path constants, */

    long double totalDist = distance(start.x - end.x, start.y - end.y);

    long double vx1 = totalDist * cos(radians(start.heading));
    long double vy1 = totalDist * sin(radians(start.heading));

    long double vx2 = totalDist * cos(radians(end.heading));
    long double vy2 = totalDist * sin(radians(end.heading));

    /* initialize parametric path, equations used in tragectory generation based of of the parametric path */

    PolynomialEquation eqx {PATH_POLYNOMIAL_ARGS(start.x, end.x, vx1, vx2)};
    PolynomialEquation eqxd = eqx.derivative();

    PolynomialEquation eqy {PATH_POLYNOMIAL_ARGS(start.y, end.y, vy1, vy2)};
    PolynomialEquation eqyd = eqy.derivative();

    CurvatureEquation c = {
        eqxd,
        eqxd.derivative(),
        eqyd,
        eqyd.derivative()
    };

    // use a trapezoidal approximation to find the path length
    long double length = (distance(eqxd.at(0), eqyd.at(0)) + distance(eqxd.at(1), eqyd.at(1))) / 2;
    for (long double t = profileDT; t < 1; t += profileDT) {
        length += distance(eqxd.at(t), eqyd.at(t));
    }
    length *= profileDT;

    // initialize data storage before trajectory generation starts
    Path profile {end, length};

    // initialize trajectory generation constants
    long double distToAccel = maxVelocity * maxVelocity / (2 * maxAcceleration);
    if (distToAccel > length / 2) {
        distToAccel = length / 2;
    }

    DistanceToTime toTime {eqxd, eqyd, profileDT};

    long double kv = 12000 / maxVelocity;

    long double distTraveled = 0;
    while (distTraveled < length) { // while path has not been fully transversed

        long double t = toTime.atDistance(distTraveled);
        // get the curvature and radius if it exists
        long double curvature = c.at(t);
        long double r = (curvature != 0 ? fabs(1 / curvature) : 0);
        // get the maximum forward velocity allowed by the curvature of the path
        long double velocityByCurvature = (curvature == 0 ? maxVelocity : (r * maxVelocity) / (r + drivetrainWidth));
        long double velocityByDistance;

        if (distTraveled < distToAccel) { // if accelerating

            // Velocity given by V^2 = V_0^2 + 2a*deltaX
            // and V = V_0 + a*t (to prevent 0 velocity forever at start
            // of path when distance = 0, *0.5 to get average between endpoints of block of time)
            // this part could use some work
            velocityByDistance = sqrt(2 * maxAcceleration * distTraveled);

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

        long double theta = atan2(eqyd.at(t), eqxd.at(t));
        // add the left and right side velocities to the profile
        profile.add(
            velocity * kv, (lVelocity - rVelocity) * kv / 2,
            eqx.at(t) + lookAheadDist * cos(theta), eqy.at(t) + lookAheadDist * sin(theta)
        );

        // update the distance traveled
        distTraveled += velocity * profileDT;
        if (distTraveled < distToAccel) {
            distTraveled += 0.5 * maxAcceleration * profileDT * profileDT;
        }

    }

    // return the completed profile
    return profile;

}