#ifdef DRIVETRAIN_HPP
#ifndef EXIT_CONDITIONS_HPP
#define EXIT_CONDITIONS_HPP

/**
 * Exit Condition template for pure pursuit motions
 * Pass args * 1000 as template arguments
 */
template <
    long maxDistance = 30000,
    long derivativeStuckThreshhold = 500,
    long maxTimeStuck = 1500
>
static bool defaultPurePursuitExit(long double curDist, bool reset) {
    static uint16_t count = 0;
    if (reset) {
        count = 0;
        return false;
    }
    if (curDist <= maxDistance / 1000.0) {
        return true;
    } else if (linearPID.getDerivative() <= derivativeStuckThreshhold / 1000.0) {
        ++count;
        if (count >= maxTimeStuck / 10) {
            return true;
        }
    } else {
        count = 0;
    }
    return false;
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
    static uint16_t count = 0;
    static uint16_t stuckCount = 0;
    if (reset) {
        count = 0;
        stuckCount = 0;
        return false;
    }
    if (firstLoop) {
        if (curDist <= maxLinearError / 1000.0) {
            return true;
        }
    }
    if (fabs(linearPID.getDerivative()) <= maxLinearDerivative / 1000.0
        && fabs(rotPID.getDerivative()) <= maxRotDerivative / 1000.0
    ) {
        ++stuckCount;
        if (stuckCount >= maxTimeStuck / 10) {
            return true;
        } else if (
            curDist <= maxLinearError / 1000.0
            && fabs(rotPID.getError()) <= maxRotError / 1000.0
        ) {
            ++count;
            if (count >= minTime / 10) {
                return true;
            }
        } else {
            count = 0;
        }
    } else {
        count = 0;
        stuckCount = 0;
    }
    return false;
}

template<
    
    long maxError = 10000,
    long maxDerivative = 2000,

    long minTime = 150,

    long maxTimeStuck = 1500

>
static bool defaultTurnExit(bool firstLoop, bool reset) {
    static uint16_t count = 0;
    static uint16_t stuckCount = 0;
    if (reset) {
        count = 0;
        stuckCount = 0;
        return false;
    }
    if (firstLoop) {
        if (fabs(rotPID.getError()) <= maxError / 1000.0) {
            return true;
        }
    }
    if (fabs(rotPID.getDerivative()) <= maxDerivative / 1000.0) {
        ++stuckCount;
        if (stuckCount >= maxTimeStuck / 10) {
            return true;
        } else if (fabs(rotPID.getError()) <= maxError / 1000.0) {
            ++count;
            if (count >= minTime / 10) {
                return true;
            }
        } else {
            count = 0;
        }
    } else {
        count = 0;
        stuckCount = 0;
    }
    return false;
}

#endif
#endif