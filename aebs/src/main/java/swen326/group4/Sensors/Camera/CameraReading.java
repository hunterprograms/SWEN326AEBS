package swen326.group4.Sensors.Camera;

/**
 * Immutable data record produced by Camera on each 50ms tick.
 *
 * Passed to CameraVoter which applies 2oo3 voting across three Camera
 * instances to produce a trusted result for AEBS Control.
 *
 * Mirrors the RadarLidarReading pattern used by Lidar and Radar.
 *
 * Requirement traceability:
 *   NF2201 : Carries per-tick classification data into the 2oo3 vote.
 *   NF4201 : Immutable - cannot be altered in transit after creation.
 */
public final class CameraReading {

    /** Detected object types for this frame - bounded, no dynamic allocation */
    private final Camera.ObjectType[] detectedObjects;

    /** Per-object confidence scores derived from weatherFactor */
    private final float[] confidenceScores;

    /** Number of valid entries in detectedObjects and confidenceScores */
    private final int count;

    /** Timestamp of this tick in milliseconds */
    private final long timestampMs;

    /**
     * Constructs a CameraReading.
     * @param detectedObjects bounded array of ObjectType, size MAX_OBJECTS
     * @param confidenceScores per-object confidence scores
     * @param count           number of valid entries
     * @param timestampMs     tick timestamp
     */
    public CameraReading(final Camera.ObjectType[] detectedObjects,
                         final float[] confidenceScores,
                         final int count,
                         final long timestampMs) {
        assert detectedObjects != null : "detectedObjects must not be null";
        assert confidenceScores != null : "confidenceScores must not be null";
        assert count >= 0 && count <= Camera.MAX_OBJECTS : "count out of valid range";
        assert timestampMs >= 0 : "timestampMs must not be negative";
        this.detectedObjects = detectedObjects;
        this.confidenceScores = confidenceScores;
        this.count = count;
        this.timestampMs = timestampMs;
    }

    public Camera.ObjectType[] detectedObjects() { return detectedObjects; }
    public float[] confidenceScores()            { return confidenceScores; }
    public int count()                           { return count; }
    public long timestampMs()                    { return timestampMs; }
}