#ifndef _EQUATIONS_HPP_
#define _EQUATIONS_HPP_

/**
 * This file contains declarations for classes utilized by the path and trajectory generation used in motion profiling
 */

namespace equations {

    // returns the distance between two points given the difference in their x and y positions
    double distance(double dx, double dy);

    /**
     * PolynomialEquation is a class for an up to five term polynomial
     *
     * This class is used to determine a polynomial function's value at a given value, and to get the derivative (as a PolynomialEquation)
     *
     * Several PolynomialEquations are used by the path and trajectory generation used in motion profiling
     */
    
    class PolynomialEquation final {
    public:

        // constructor, with coefficients of terms followed by the power that respective term is raised to
        PolynomialEquation(long double a, int pow1, long double b, int pow2, long double c,
            int pow3, long double d, int pow4, long double e, int pow5);

        // returns the value of the function at the given point
        long double at(long double value) const;

        // returns the derivative PolynomialEquation
        PolynomialEquation derivative() const;

    private:

        // coefficients of the terms
        const long double a;
        const long double b;
        const long double c;
        const long double d;
        const long double e;

        // powers the terms are raised to
        const int pow1;
        const int pow2;
        const int pow3;
        const int pow4;
        const int pow5;

    };
    
    /**
     * CurvatureEquation is used to find the curvature at a given value of a parametric function
     *
     * CurvatureEquation is utilized by trajectory generation used in motion profiling to properly limit speed while arcing
     */

    class CurvatureEquation final {
    public:

        // constructor, takes references to the derivative and second derivative of the x and y components 
        CurvatureEquation(const PolynomialEquation& xd, const PolynomialEquation& xdd,
            const PolynomialEquation& yd, const PolynomialEquation& ydd);

        // returns the curvature (which is equivalent to 1 / r) at a current value along a parametric curve 
        long double at(long double value) const;

    private:

        // PolynomialEquations used to compute the curvature
        // For optimization, the second derivatives are not references, as they are not utilized anywhere else
        // by the path or trajectory generation used by motion profiling
        const PolynomialEquation& xd;
        const PolynomialEquation xdd;
        const PolynomialEquation& yd;
        const PolynomialEquation ydd;

    };

    /**
     * DistanceToTime is a class used to find the parametric parameter at which a certain length of the curve has been traveled
     *
     * For optimization, this class assumes that the distance is always increasing
     */
    
    class DistanceToTime final {
    public:

        // constructor, takes the first derivatives of the x and y components of a parametric equation as well as a time step
        // a higher time step will yield a more accurate answer at the expense of computation time
        DistanceToTime(const PolynomialEquation& xd, const PolynomialEquation& yd, long double step);

        long double atDistance(long double dist);

    private:

        const PolynomialEquation& xd;
        const PolynomialEquation& yd;
        const long double step;

        long double time;
        long double accumulatedDistance;

    };

} // namespace equations

#endif
