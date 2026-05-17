package Simulator.ScenarioGenerator;

/**
 * Mutable physics state of the ego vehicle.
 *
 * Maintained by ScenarioGenerator and updated each simulation step.
 * All sensor outputs are derived from this state combined with WorldObject
 * positions.
 *
 * Wheel RPM is derived from vehicle speed:
 *   rpm = (speedKmh / 3.6) / (2 * PI * WHEEL_RADIUS_M) * 60
 *
 * A standard passenger car tyre has an effective rolling radius of ~0.316m.
 */
public class VehicleState {

    // -------------------------------------------------------------------------
    // Tyre constants
    // -------------------------------------------------------------------------

    /** Effective rolling radius of a standard passenger tyre in metres */
    private static final double WHEEL_RADIUS_M = 0.316;

    /** Circumference of the tyre */
    private static final double WHEEL_CIRCUMFERENCE_M = 2.0 * Math.PI * WHEEL_RADIUS_M;

    // -------------------------------------------------------------------------
    // Mutable state
    // -------------------------------------------------------------------------

    /** Current speed in km/h */
    private double speedKmh;

    /**
     * Current acceleration in m/s² — positive = speeding up, negative = braking.
     * Zero = coasting.
     */
    private double accelerationMps2;

    /**
     * Current turn rate in degrees per second.
     * Positive = turning right, negative = turning left, zero = straight.
     */
    private double turnRateDegPerSec;

    /**
     * Accumulated heading change from scenario start in degrees.
     * Used to track cumulative direction for bearing adjustments.
     */
    private double headingDegrees;

    /**
     * Maximum speed cap for acceleration events.
     * 0 = no cap. Set by ACCELERATE events.
     */
    private double speedCapKmh;

    /** Braking is active (deceleration in progress) */
    private boolean braking;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param initialSpeedKmh  starting speed of the ego vehicle
     */
    public VehicleState(final double initialSpeedKmh) {
        this.speedKmh          = initialSpeedKmh;
        this.accelerationMps2  = 0.0;
        this.turnRateDegPerSec = 0.0;
        this.headingDegrees    = 0.0;
        this.speedCapKmh       = 0.0;
        this.braking           = false;
    }

    // -------------------------------------------------------------------------
    // Physics update
    // -------------------------------------------------------------------------

    /**
     * Advances vehicle state by one time step.
     *
     * @param dtSeconds  time step duration in seconds
     * @return           distance advanced in metres this step
     */
    public double advance(final double dtSeconds) {
        // Update speed
        if (accelerationMps2 != 0.0) {
            final double deltaKmh = accelerationMps2 * dtSeconds * 3.6;
            speedKmh += deltaKmh;

            // Clamp to zero — vehicle doesn't reverse
            if (speedKmh < 0.0) {
                speedKmh = 0.0;
                accelerationMps2 = 0.0;
                braking = false;
            }

            // Clamp to speed cap for acceleration
            if (speedCapKmh > 0.0 && speedKmh > speedCapKmh) {
                speedKmh = speedCapKmh;
                accelerationMps2 = 0.0;
            }
        }

        // Update heading
        headingDegrees += turnRateDegPerSec * dtSeconds;

        // Distance advanced this step (average speed * time, simplified)
        return (speedKmh / 3.6) * dtSeconds;
    }

    // -------------------------------------------------------------------------
    // Wheel RPM derivation
    // -------------------------------------------------------------------------

    /**
     * Returns wheel RPM derived from current speed.
     * All four wheels share the same RPM (no differential model needed).
     *
     * @param noiseLevel  fraction of noise to add (0 = perfect)
     * @param rng         random number generator for noise
     */
    public float[] getWheelRpm(final float noiseLevel, final java.util.Random rng) {
        final double speedMs = speedKmh / 3.6;
        final double baseRpm = (speedMs / WHEEL_CIRCUMFERENCE_M) * 60.0;
        final float[] rpm = new float[4];
        for (int i = 0; i < 4; i++) {
            final double noise = (rng.nextDouble() * 2.0 - 1.0) * noiseLevel * baseRpm * 0.01;
            rpm[i] = (float) Math.max(0.0, baseRpm + noise);
        }
        return rpm;
    }

    // -------------------------------------------------------------------------
    // Event application
    // -------------------------------------------------------------------------

    public void setAcceleration(final double mps2) {
        this.accelerationMps2 = mps2;
        this.braking = mps2 < 0.0;
        this.speedCapKmh = 0.0;
    }

    public void setAcceleration(final double mps2, final double capKmh) {
        this.accelerationMps2 = mps2;
        this.braking = mps2 < 0.0;
        this.speedCapKmh = capKmh;
    }

    public void setSpeed(final double kmh) {
        this.speedKmh = Math.max(0.0, kmh);
        this.accelerationMps2 = 0.0;
        this.braking = false;
    }

    public void setTurnRate(final double degPerSec) {
        this.turnRateDegPerSec = degPerSec;
    }

    public void coast() {
        this.accelerationMps2 = 0.0;
        this.braking = false;
        this.speedCapKmh = 0.0;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    public double getSpeedKmh()          { return speedKmh; }
    public double getAccelerationMps2()  { return accelerationMps2; }
    public double getTurnRateDegPerSec() { return turnRateDegPerSec; }
    public double getHeadingDegrees()    { return headingDegrees; }
    public boolean isBraking()           { return braking; }
}
