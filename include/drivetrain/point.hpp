#ifdef DRIVETRAIN_HPP
#ifndef POINT_HPP
#define POINT_HPP

struct Drivetrain::XYPoint {
    long double x;
    long double y;
};

struct Drivetrain::Point {

    Point(long double x, long double y, long double heading);

    Point& withAction(std::function<void()>&& action, double dist, bool duringTurn = false);

    long double x;
    long double y;
    long double heading;

};

struct Drivetrain::Waypoint {

    Waypoint(long double x, long double y);

    Waypoint& withAction(std::function<void()>&& action, double dist, bool duringTurn = false);
    Waypoint& withLookAhead(long double newLookAhead);

    long double x;
    long double y;
    long double lookAheadDistance {defaultLookAheadDistance};

};

#endif
#endif