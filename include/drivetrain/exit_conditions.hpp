#ifdef _DRIVETRAIN_HPP_
#ifndef _EXIT_CONDITIONS_HPP_
#define _EXIT_CONDITIONS_HPP_

/**
 * Separate file to define default exit condition functions
 * included in the Drivetrain class in drivetrain.hpp
 *
 * Functions take a reset parameter to determine whether to reset static variables
 */

/**
 * Exit Condition template for pure pursuit motions
 * Pass args * 1000 as template arguments
 */

template <
    long derivativeStuckThreshold = 500,
    long maxTimeStuck = 1500
>
static bool defaultPurePursuitExit(long double curDist, long double lookAheadDist, bool reset) {
    // track time stuck 
    static uint16_t count       = 0;
    static uint32_t noiseCount  = 0;
    if (reset) {
        count = 0;
        noiseCount = 0;
        return false;
    }
    if (curDist <= lookAheadDist) {
        return true; // exit if within look ahead distance of target
    } else if (linearPID.getDerivative() <= derivativeStuckThreshold / 1000.0) {
        ++count; // increment time stuck
        count += noiseCount; // increment by time where derivative noise disrupted the exit conditions
        noiseCount = 0; // derivative noise is not currently significant enough to wrongfully stop exit
        if (count >= maxTimeStuck / 10) {
            return true; // exit if stuck
        }
    } else if (count > 0) {
        ++noiseCount; // account for derivative noise
        if (noiseCount >= 5) { // if derivative is likely indeed outside acceptable thresholds, reset the counts
            count = 0;
            noiseCount = 0;
        }
    }
    return false; // not close enough to target / did not time out
}

/**
 * Exit Condition template for linear motions
 * Pass args * 1000 as template arguments
 */

template <
    
    long maxLinearError = 1000,
    long maxLinearDerivative = 500,
    
    long maxRotError = 10000,
    long maxRotDerivative = 2000,

    long minTime = 150,

    long maxTimeStuck = 1500

>
static bool defaultLinearExit(long double curDist, bool firstLoop, bool reset) {
    // track time stuck
    static uint16_t count       = 0;
    static uint16_t stuckCount  = 0;
    static uint16_t noiseCount  = 0;
    if (reset) {
        count = 0;
        stuckCount = 0;
        noiseCount = 0;
        return false;
    }
    if (firstLoop) {
        if (curDist <= maxLinearError / 1000.0) {
            return true; // exit if starting sufficiently close to target
        }
    }
    if (fabs(linearPID.getDerivative()) <= maxLinearDerivative / 1000.0
        && fabs(rotPID.getDerivative()) <= maxRotDerivative / 1000.0
    ) {
        ++stuckCount; // increment time stuck
        stuckCount += noiseCount; // increment by time where derivative noise disrupted the exit conditions
        if (stuckCount >= maxTimeStuck / 10) {
            return true; // exit if stuck
        } else if (
            curDist <= maxLinearError / 1000.0
            && fabs(rotPID.getError()) <= maxRotError / 1000.0
        ) {
            ++count; // increment time near target
            count += noiseCount; // increment by time where derivative noise disrupted the exit conditions
            if (count >= minTime / 10) {
                return true; // exit if settled (derivative evaluated earlier) near target
            }
        } else {
            count = 0;
        }
        noiseCount = 0; // derivative noise is not currently significant enough to wrongfully stop exit
    } else if (stuckCount > 0) {
        ++noiseCount; // account for derivative noise
        if (noiseCount >= 5) { // if derivative is likely indeed outside acceptable thresholds, reset the counts
            count = 0;
            stuckCount = 0;
            noiseCount = 0;
        }
    }
    return false;
}

/**
 * Exit Condition template for turns
 * Pass args * 1000 as template arguments
 */

template<
    
    long maxError = 10000,
    long maxDerivative = 2000,

    long minTime = 150,

    long maxTimeStuck = 1500

>
static bool defaultTurnExit(bool firstLoop, bool reset) {
    // track time stuck
    static uint16_t count       = 0;
    static uint16_t stuckCount  = 0;
    static uint16_t noiseCount  = 0;
    if (reset) {
        count = 0;
        stuckCount = 0;
        noiseCount = 0;
        return false;
    }
    if (firstLoop) {
        if (fabs(rotPID.getError()) <= maxError / 1000.0) {
            return true; // exit if starting sufficiently close to target
        }
    }
    if (fabs(rotPID.getDerivative()) <= maxDerivative / 1000.0) {
        ++stuckCount; // increment time stuck
        stuckCount += noiseCount; // increment by time where derivative noise disrupted the exit conditions
        if (stuckCount >= maxTimeStuck / 10) {
            return true; // exit if stuck
        } else if (fabs(rotPID.getError()) <= maxError / 1000.0) {
            ++count;
            count += noiseCount; // increment by time where derivative noise disrupted the exit conditions
            if (count >= minTime / 10) {
                return true; // exit if settled (derivative evaluated earlier) near target
            }
        } else {
            count = 0;
        }
        noiseCount = 0; // derivative noise is not currently significant enough to wrongfully stop exit
    } else if (stuckCount > 0) {
        ++noiseCount; // account for derivative noise
        if (noiseCount >= 5) { // if derivative is likely indeed outside acceptable thresholds, reset the counts
            count = 0;
            stuckCount = 0;
            noiseCount = 0;
        }
    }
    return false;
}

#endif
#endif