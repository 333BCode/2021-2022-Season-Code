#include "util/equations.hpp"

#include <cmath>

namespace equations {

double distance(double dx, double dy) {
    return sqrt(dx * dx + dy * dy);
}

PolynomialEquation::PolynomialEquation(long double a, int pow1, long double b, int pow2, long double c,
    int pow3, long double d, int pow4, long double e, int pow5)
    : a {a}, b {b}, c {c}, d {d}, e {e},
    pow1 {pow1}, pow2 {pow2}, pow3 {pow3}, pow4 {pow4}, pow5 {pow5} {}

long double PolynomialEquation::at(long double value) const {
    return a * pow(value, pow1) +  b * pow(value, pow2) + c * pow(value, pow3)
        + d * pow(value, pow4) + e * pow(value, pow5);
}

PolynomialEquation PolynomialEquation::derivative() const {
    return {
        a * pow1, pow1 != 0 ? pow1 - 1 : 0,
        b * pow2, pow2 != 0 ? pow2 - 1 : 0,
        c * pow3, pow3 != 0 ? pow3 - 1 : 0,
        d * pow4, pow4 != 0 ? pow4 - 1 : 0,
        e * pow5, pow5 != 0 ? pow5 - 1 : 0
    };
}

CurvatureEquation::CurvatureEquation(const PolynomialEquation& xd, const PolynomialEquation& xdd,
    const PolynomialEquation& yd, const PolynomialEquation& ydd)
    : xd {xd}, xdd {xdd}, yd {yd}, ydd {ydd} {}

long double CurvatureEquation::at(long double value) const {
    return (xd.at(value) * ydd.at(value) - xdd.at(value) * yd.at(value))
        / pow(pow(xd.at(value), 2) + pow(yd.at(value), 2), 1.5);
}

DistanceToTime::DistanceToTime(const PolynomialEquation& xd, const PolynomialEquation& yd, long double step)
    : xd {xd}, yd {yd}, step {step}, time {0} {}

long double DistanceToTime::atDistance(long double dist) {

    while (accumulatedDistance * step < dist && time < 1) {
        accumulatedDistance += (distance(xd.at(time), yd.at(time))
            + distance(xd.at(time + step), yd.at(time + step))) / 2;
        time += step;
    }

    return time;

}

} // namespace equations