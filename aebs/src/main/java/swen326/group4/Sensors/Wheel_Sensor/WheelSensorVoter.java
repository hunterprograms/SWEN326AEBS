package swen326.group4.Sensors.Wheel_Sensor;

/**
 * WheelSensorVoter
 *
 * Implements 2oo3 (two-out-of-three) voting across three redundant
 * WheelSensor instances (FR2208).
 *
 * For each wheel, the voter compares the three sensor readings and returns
 * the average of the two (or more) that agree within a defined tolerance.
 * If no two sensors agree, the wheel is considered unresolvable and -1 is
 * returned for that wheel.
 *
 * Readings flagged as unavailable by a sensor (value -1) are excluded from
 * voting automatically.
 */
public class WheelSensorVoter {

    /* ------------------------------------------------------------------ */
    /* Constants                                                            */
    /* ------------------------------------------------------------------ */

    /** Number of wheels. */
    private static final int WHEEL_COUNT = 4;

    /**
     * Maximum RPM difference between two sensors for them to be considered
     * in agreement.
     */
    private static final float AGREEMENT_THRESHOLD_RPM = 50.0f;

    /** Sentinel value indicating a wheel reading is unavailable. */
    private static final float UNAVAILABLE = -1.0f;

    /* ------------------------------------------------------------------ */
    /* Fields                                                               */
    /* ------------------------------------------------------------------ */

    private final WheelSensor sensorA;
    private final WheelSensor sensorB;
    private final WheelSensor sensorC;

    /* ------------------------------------------------------------------ */
    /* Constructor                                                          */
    /* ------------------------------------------------------------------ */

    /**
     * @param sensorA  first sensor in the redundant trio
     * @param sensorB  second sensor in the redundant trio
     * @param sensorC  third sensor in the redundant trio
     */
    public WheelSensorVoter(final WheelSensor sensorA,
                             final WheelSensor sensorB,
                             final WheelSensor sensorC) {
        this.sensorA = sensorA;
        this.sensorB = sensorB;
        this.sensorC = sensorC;
    }

    /* ------------------------------------------------------------------ */
    /* Public API                                                           */
    /* ------------------------------------------------------------------ */

    /**
     * Advances all three sensors by one 10ms frame, then performs 2oo3
     * voting on the results.
     *
     * @return float[4] of voted RPM values per wheel, or -1 for any wheel
     *         that could not be resolved
     */
    public float[] vote() {
        sensorA.readRPM();
        sensorB.readRPM();
        sensorC.readRPM();

        final WheelSensorData dataA = sensorA.getLatestReading();
        final WheelSensorData dataB = sensorB.getLatestReading();
        final WheelSensorData dataC = sensorC.getLatestReading();

        final float[] result = new float[WHEEL_COUNT];

        for (int i = 0; i < WHEEL_COUNT; i++) {
            final float a = safeRpm(dataA, i);
            final float b = safeRpm(dataB, i);
            final float c = safeRpm(dataC, i);
            result[i] = resolveWheel(a, b, c);
        }

        return result;
    }

    /**
     * Returns true if all three sensors still have frames remaining.
     */
    public boolean hasData() {
        return sensorA.hasData() && sensorB.hasData() && sensorC.hasData();
    }

    /* ------------------------------------------------------------------ */
    /* Private helpers                                                      */
    /* ------------------------------------------------------------------ */

    /**
     * Safely extracts the RPM for a given wheel from a reading, returning
     * UNAVAILABLE if the reading itself is null.
     */
    private float safeRpm(final WheelSensorData data, final int wheelIndex) {
        if (data == null) {
            return UNAVAILABLE;
        }
        return data.getRpm(wheelIndex);
    }

    /**
     * Applies 2oo3 voting logic for a single wheel across three readings.
     *
     * Agreement is checked pairwise. The first agreeing pair found wins and
     * their average is returned. If no pair agrees, UNAVAILABLE is returned.
     *
     * Unavailable readings (-1) are never averaged into the result.
     */
    private float resolveWheel(final float a, final float b, final float c) {
        final boolean abAgree = agree(a, b);
        final boolean acAgree = agree(a, c);
        final boolean bcAgree = agree(b, c);

        if (abAgree && acAgree) {
            /* All three agree — average all three. */
            return (a + b + c) / 3.0f;
        }
        if (abAgree) {
            return (a + b) / 2.0f;
        }
        if (acAgree) {
            return (a + c) / 2.0f;
        }
        if (bcAgree) {
            return (b + c) / 2.0f;
        }

        /* No two sensors agree — wheel unresolvable. */
        return UNAVAILABLE;
    }

    /**
     * Returns true if both values are available and within the agreement
     * threshold of each other.
     */
    private boolean agree(final float x, final float y) {
        if (x == UNAVAILABLE || y == UNAVAILABLE) {
            return false;
        }
        return Math.abs(x - y) <= AGREEMENT_THRESHOLD_RPM;
    }
}