#ifdef _DRIVETRAIN_HPP_
#ifndef _POINT_HPP_
#define _POINT_HPP_

/**
 * Separate file for Point structs that are a part of Drivetrain
 * included in drivetrain.hpp
 */

// simple (x, y) coord data
struct Drivetrain::XYPoint {
    long double x;
    long double y;
};

// Stores (x, y) location and heading
// Allows storing actions to be executed during the next movement
struct Drivetrain::Point {

    Point(long double x, long double y, long double heading);

    // Store action
    Point& withAction(std::function<void()>&& action, double dist, bool duringTurn = false);

    long double x;
    long double y;
    long double heading;

};

// Stores location data for pure pursuit
struct Drivetrain::Waypoint {

    Waypoint(long double x, long double y);

    // Store actions to be executed during the next movement (or pure pursuit segment)
    Waypoint& withAction(std::function<void()>&& action, double dist, bool duringTurn = false);
    // Change the look ahead distance for the movement to this point
    Waypoint& withLookAhead(long double newLookAhead);
    // Change the exit conditions for the movement to this point
    Waypoint& withExitCondition(PurePursuitExitConditions newExitConditions);

    long double x;
    long double y;
    long double lookAheadDistance {defaultLookAheadDistance};
    PurePursuitExitConditions exitConditions {defaultPurePursuitExit};

};

#endif
#endif