#ifndef EQUATION_HPP
#define EQUATION_HPP

namespace equations {

double distance(double dx, double dy);

class PolynomialEquation final {
public:

    PolynomialEquation(long double a, int pow1, long double b, int pow2, long double c,
        int pow3, long double d, int pow4, long double e, int pow5);

    long double at(long double value) const;

    PolynomialEquation derivative() const;

private:

    const long double a;
    const long double b;
    const long double c;
    const long double d;
    const long double e;

    const int pow1;
    const int pow2;
    const int pow3;
    const int pow4;
    const int pow5;

};

class CurvatureEquation final {
public:

    CurvatureEquation(const PolynomialEquation& xd, const PolynomialEquation& xdd,
        const PolynomialEquation& yd, const PolynomialEquation& ydd);

    long double at(long double value) const;

private:

    const PolynomialEquation& xd;
    const PolynomialEquation& xdd;
    const PolynomialEquation& yd;
    const PolynomialEquation& ydd;

};

class DistanceToTime {
public:

    DistanceToTime(const PolynomialEquation& xd, const PolynomialEquation& yd, long double step);

    long double atDistance(long double dist);

private:

    const PolynomialEquation& xd;
    const PolynomialEquation& yd;
    const long double step;

    long double time;
    long double accumulatedDistance {0};

};

} // namespace equations

#endif