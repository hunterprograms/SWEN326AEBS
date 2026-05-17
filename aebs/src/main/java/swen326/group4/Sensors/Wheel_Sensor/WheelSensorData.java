package swen326.group4.Sensors.Wheel_Sensor;

import java.util.Arrays;

/**
 * WheelSensorData
 *
 * Immutable data container for a single validated wheel speed reading
 * produced by WheelSensor.
 *
 * Holds the timestamp, sensor ID, and RPM value for each of the four
 * wheels. A wheel RPM of -1 indicates the reading for that wheel was
 * unavailable at the time of capture.
 *
 * Wheel index order: 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
 */
public class WheelSensorData {

    /* ------------------------------------------------------------------ */
    /* Constants                                                            */
    /* ------------------------------------------------------------------ */

    private static final int WHEEL_COUNT = 4;

    private static final String[] WHEEL_NAMES = {
        "front_left", "front_right", "rear_left", "rear_right"
    };

    /** Sentinel value indicating a wheel reading is unavailable. */
    public static final float UNAVAILABLE = -1.0f;

    /* ------------------------------------------------------------------ */
    /* Fields                                                               */
    /* ------------------------------------------------------------------ */

    /** Simulated timestamp in milliseconds when this reading was taken. */
    private final int timestampMs;

    /** ID of the sensor set that produced this reading (1, 2, or 3). */
    private final int sensorId;

    /** Validated RPM for each wheel; -1 if unavailable. */
    private final float[] rpm;

    /* ------------------------------------------------------------------ */
    /* Constructor                                                          */
    /* ------------------------------------------------------------------ */

    /**
     * @param timestampMs  simulated time of the reading in milliseconds
     * @param rpm          float[4] of validated RPM values; -1 = unavailable
     * @param sensorId     which sensor set produced this reading (1, 2, or 3)
     */
    public WheelSensorData(final int timestampMs, final float[] rpm, final int sensorId) {
        if (rpm.length != WHEEL_COUNT) {
            throw new IllegalArgumentException(
                "rpm array must have exactly " + WHEEL_COUNT + " elements"
            );
        }
        this.timestampMs = timestampMs;
        this.sensorId = sensorId;
        this.rpm = Arrays.copyOf(rpm, WHEEL_COUNT);
    }

    /* ------------------------------------------------------------------ */
    /* Getters                                                              */
    /* ------------------------------------------------------------------ */

    /**
     * Returns the RPM for the given wheel index, or UNAVAILABLE (-1) if
     * the reading was not accepted for that wheel.
     *
     * @param wheelIndex  0=front_left, 1=front_right, 2=rear_left, 3=rear_right
     */
    public float getRpm(final int wheelIndex) {
        return rpm[wheelIndex];
    }

    /** Returns a defensive copy of all four wheel RPM values. */
    public float[] getAllRpm() {
        return Arrays.copyOf(rpm, WHEEL_COUNT);
    }

    /** Returns the simulated timestamp in milliseconds. */
    public int getTimestampMs() {
        return timestampMs;
    }

    /** Returns the sensor set ID (1, 2, or 3). */
    public int getSensorId() {
        return sensorId;
    }

    /**
     * Returns true if the given wheel has a valid (non-unavailable) reading.
     *
     * @param wheelIndex  0–3
     */
    public boolean isWheelAvailable(final int wheelIndex) {
        return rpm[wheelIndex] != UNAVAILABLE;
    }

    /**
     * Returns true if at least one wheel has a valid reading.
     */
    public boolean hasAnyValidReading() {
        for (final float r : rpm) {
            if (r != UNAVAILABLE) {
                return true;
            }
        }
        return false;
    }

    /* ------------------------------------------------------------------ */
    /* Object overrides                                                     */
    /* ------------------------------------------------------------------ */

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder();
        sb.append("WheelSensorData{")
          .append("sensor=").append(sensorId)
          .append(", t=").append(timestampMs).append("ms")
          .append(", rpm=[");
        for (int i = 0; i < WHEEL_COUNT; i++) {
            sb.append(WHEEL_NAMES[i]).append(":");
            if (rpm[i] == UNAVAILABLE) {
                sb.append("N/A");
            } else {
                sb.append(String.format("%.1f", rpm[i]));
            }
            if (i < WHEEL_COUNT - 1) {
                sb.append(", ");
            }
        }
        sb.append("]}");
        return sb.toString();
    }
}