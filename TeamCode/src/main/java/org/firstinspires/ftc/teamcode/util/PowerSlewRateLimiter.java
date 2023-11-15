package org.firstinspires.ftc.teamcode.util;

/**
 * A slew rate limiter for motor power etc
 *
 * Rate limits are specified in change/sec
 *
 * The accelRateLimit is applied when value increasing away from zero
 * The decelRateLimit is applied when value is decreasing towards zero
 *
 */
public class PowerSlewRateLimiter {
    private final double m_accelRateLimit;
    private final double m_decelRateLimit;
    private double m_prevVal;
    private double m_prevTime;
    public PowerSlewRateLimiter(double accelRateLimit, double decelRateLimit, double initialValue) {
        m_accelRateLimit = accelRateLimit;
        m_decelRateLimit = decelRateLimit;
        m_prevVal = initialValue;
        m_prevTime = getTimestamp();
    }

        static double getTimestamp() {
            return (double) System.nanoTime() / 1E9;
        }
    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public PowerSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        boolean accel = Math.abs(input) > Math.abs(m_prevVal);
        double limit = accel ? m_accelRateLimit : m_decelRateLimit;

        m_prevVal +=
                com.arcrobotics.ftclib.util.MathUtils.clamp(
                        input - m_prevVal,
                        -limit * elapsedTime,
                        limit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = getTimestamp();
    }
}
