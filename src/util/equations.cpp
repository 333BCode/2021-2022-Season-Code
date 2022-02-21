#include "util/equations.hpp"

#include <cmath>

namespace equations {

    // returns the distance between two points given the difference in their x and y positions
    double distance(double dx, double dy) {
        return sqrt(dx * dx + dy * dy); // more performant than std::hypot
    }

    // constructor, with coefficients of terms followed by the power that respective term is raised to
    PolynomialEquation::PolynomialEquation(long double a, int pow1, long double b, int pow2, long double c,
        int pow3, long double d, int pow4, long double e, int pow5)
        : a {a}, b {b}, c {c}, d {d}, e {e},
        pow1 {pow1}, pow2 {pow2}, pow3 {pow3}, pow4 {pow4}, pow5 {pow5} {}

    // returns the value of the function at the given point
    long double PolynomialEquation::at(long double value) const {
        return a * pow(value, pow1) +  b * pow(value, pow2) + c * pow(value, pow3)
            + d * pow(value, pow4) + e * pow(value, pow5);
    }

    // returns the derivative PolynomialEquation
    PolynomialEquation PolynomialEquation::derivative() const {
        return { // ternary operator used to prevent possible divide by 0 when using higher order derivatives
            a * pow1, pow1 != 0 ? pow1 - 1 : 0,
            b * pow2, pow2 != 0 ? pow2 - 1 : 0,
            c * pow3, pow3 != 0 ? pow3 - 1 : 0,
            d * pow4, pow4 != 0 ? pow4 - 1 : 0,
            e * pow5, pow5 != 0 ? pow5 - 1 : 0
        };
    }

    // constructor, takes references to the derivative and second derivative of the x and y components
    CurvatureEquation::CurvatureEquation(const PolynomialEquation& xd, const PolynomialEquation& xdd,
        const PolynomialEquation& yd, const PolynomialEquation& ydd)
        : xd {xd}, xdd {xdd}, yd {yd}, ydd {ydd} {}

    // returns the curvature (which is equivalent to 1 / r) at a current value along a parametric curve
    long double CurvatureEquation::at(long double value) const {
        return (xd.at(value) * ydd.at(value) - xdd.at(value) * yd.at(value))
            / pow(pow(xd.at(value), 2) + pow(yd.at(value), 2), 1.5);
    }

    // constructor, takes the first derivatives of the x and y components of a parametric equation as well as a time step
    // a higher time step will yield a more accurate answer at the expense of computation time
    DistanceToTime::DistanceToTime(const PolynomialEquation& xd, const PolynomialEquation& yd, long double step)
        : xd {xd}, yd {yd}, step {step}, time {0}, accumulatedDistance {0} {}

    // Converts a distance along a parametric path to the value of the parametric parameter
    // at which that distance has been traveled (starting from t = 0)
    // For optimization, it is assumed that the values of dist subsequently
    // passed to atDistance of an instance of DistanceToTime are always increasing
    // A value from 0 to 1 is returned
    long double DistanceToTime::atDistance(long double dist) {

        while (accumulatedDistance * step < dist && time < 1) { // while not enough of the path has been transversed
            // approximate the distance transversed using a trapezoidal approximation
            // dt is accounted for by multiplying accumulatedDistance by step in the while condition
            accumulatedDistance += (distance(xd.at(time), yd.at(time))
                + distance(xd.at(time + step), yd.at(time + step))) / 2;
            // increment the parametric parameter since we have not yet reached the target
            time += step;
        }

        // return the parametric parameter
        return time;

    }

} // namespace equations