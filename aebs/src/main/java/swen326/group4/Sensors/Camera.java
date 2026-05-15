package swen326.group4.Sensors;

/**
 * Represents a single camera sensor unit in the AEBS.
 *
 * Three instances of this class (IDs 1, 2, 3) feed into a CameraVoter
 * which applies 2oo3 voting to produce a trusted classification result.
 */
public class Camera {
 
    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------
 
    /**
     * Classification of detected road objects.
     * UNKNOWN is used when the camera cannot confidently classify an object.
     * NONE means no object is present in that slot.
     */
    public enum ObjectType {
        NONE,
        VEHICLE,
        PEDESTRIAN,
        CYCLIST,
        STATIONARY_OBJECT,
        UNKNOWN
    }
 
    /**
     * Current light condition affecting classification accuracy.
     * Feeds into confidence score degradation logic.
     */
    public enum LightCondition {
        BRIGHT,
        NORMAL,
        LOW,
        NIGHT
    }
 
    /**
     * Current weather condition affecting classification accuracy.
     * SEVERE triggers the NF2202 fallback mechanism.
     */
    public enum WeatherCondition {
        CLEAR,
        RAIN,
        FOG,
        SNOW,
        SEVERE   // triggers obstruction fallback per NF2202
    }
 
    /**
     * Operational state of this camera unit.
     * Drives the fallback and fault escalation behaviour.
     */
    public enum CameraState {
        OPERATIONAL,      // normal function
        DEGRADED,         // reduced accuracy - still contributing to 2oo3 vote
        OBSTRUCTED,       // NF2202: fallback triggered, output excluded from vote
        FAILED            // hardware failure - excluded from vote, driver alerted
    }
 
    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
 
    /** Update frequency per spec section 4.2: every 50ms */
    public static final int UPDATE_INTERVAL_MS = 50;
 
    /** Maximum number of objects tracked per frame - bounded per Power of Ten rule 2 */
    public static final int MAX_OBJECTS = 10;
 
    /** Confidence threshold below which a reading is treated as unreliable (NF4201) */
    public static final double CORRUPTION_THRESHOLD = 0.2;
 
    /** Confidence threshold below which the camera is considered degraded (NF2202) */
    public static final double DEGRADED_THRESHOLD = 0.5;
 
    /** Valid confidence range */
    public static final double MIN_CONFIDENCE = 0.0;
    public static final double MAX_CONFIDENCE = 1.0;
 
    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------
 
    /** Sensor ID: 1, 2, or 3 — matches Camera(1), Camera(2), Camera(3) in architecture */
    private final int sensorId;
 
    /** Timestamp of the most recent update in milliseconds */
    private long timestampMs;
 
    /** Current operational state of this camera unit */
    private CameraState state;
 
    /** Current light condition - affects classification confidence */
    private LightCondition lightCondition;
 
    /** Current weather condition - SEVERE triggers NF2202 fallback */
    private WeatherCondition weatherCondition;
 
    /** Detected object types for this frame - bounded array, no dynamic allocation */
    private final ObjectType[] detectedObjects;
 
    /** Per-object confidence scores (0.0 to 1.0) */
    private final double[] confidenceScores;
 
    /** Number of objects detected in this frame */
    private int detectedCount;
 
    /**
     * Whether this reading has been flagged as corrupted (NF4201).
     * A corrupted reading is excluded from the 2oo3 vote in CameraVoter.
     */
    private boolean dataCorrupted;
 
    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
 
    /**
     * Constructs a Camera sensor unit.
     * @param sensorId 1, 2, or 3 — must match the architecture diagram
     */
    public Camera(final int sensorId) {
        assert sensorId >= 1 && sensorId <= 3 : "sensorId must be 1, 2, or 3";
        assert MAX_OBJECTS > 0 : "MAX_OBJECTS must be positive";
        this.sensorId = sensorId;
        this.state = CameraState.OPERATIONAL;
        this.lightCondition = LightCondition.NORMAL;
        this.weatherCondition = WeatherCondition.CLEAR;
        this.detectedObjects = new ObjectType[MAX_OBJECTS];
        this.confidenceScores = new double[MAX_OBJECTS];
        this.detectedCount = 0;
        this.timestampMs = 0;
        this.dataCorrupted = false;
 
        // Initialise arrays to safe defaults - Power of Ten rule 2
        for (int i = 0; i < MAX_OBJECTS; i++) {
            this.detectedObjects[i] = ObjectType.NONE;
            this.confidenceScores[i] = 0.0;
        }
    }
 
    // -------------------------------------------------------------------------
    // Core update method - called by simulator every 50ms
    // -------------------------------------------------------------------------
 
    /**
     * Receives a new sensor reading from the simulator.
     * Validates data integrity (NF4201) and updates operational state (NF2202).
     *
     * @param timestampMs   timestamp of this reading
     * @param objects       array of detected object types
     * @param confidence    array of confidence scores matching objects
     * @param count         number of valid entries in the arrays
     * @return true if the update was accepted; false if rejected due to invalid input or failure state
     */
    public boolean update(final long timestampMs,
                          final ObjectType[] objects,
                          final double[] confidence,
                          final int count) {
 
        assert timestampMs >= 0 : "timestampMs must not be negative";
        assert count >= 0 : "count must not be negative";
 
        // Reject updates if camera has fully failed
        if (this.state == CameraState.FAILED) {
            return false;
        }
 
        // Validate inputs - NF4201 corruption detection
        if (objects == null || confidence == null) {
            this.dataCorrupted = true;
            return false;
        }
        if (count < 0 || count > MAX_OBJECTS) {
            this.dataCorrupted = true;
            return false;
        }
 
        this.timestampMs = timestampMs;
        this.detectedCount = count;
        this.dataCorrupted = false;
 
        for (int i = 0; i < count; i++) {
            // Clamp confidence to valid range - NF4201
            final double clampedConf = clampConfidence(confidence[i]);
            this.detectedObjects[i] = (objects[i] != null) ? objects[i] : ObjectType.UNKNOWN;
            this.confidenceScores[i] = clampedConf;
 
            // Flag as corrupted if any object has suspiciously low confidence
            if (clampedConf < CORRUPTION_THRESHOLD) {
                this.dataCorrupted = true;
            }
        }
 
        // Update operational state based on environment and data quality
        updateState();
 
        return true;
    }
 
    // -------------------------------------------------------------------------
    // State management - NF2202 fallback logic
    // -------------------------------------------------------------------------
 
    /**
     * Updates the camera's operational state based on weather, light, and data quality.
     * SEVERE weather or corrupted data triggers the NF2202 fallback.
     */
    private void updateState() {
        assert this.state != null : "state must not be null before updateState()";
        assert this.weatherCondition != null : "weatherCondition must not be null before updateState()";
 
        if (this.state == CameraState.FAILED) {
            return; // FAILED is a terminal state - only hardware reset can recover
        }
 
        if (this.weatherCondition == WeatherCondition.SEVERE || this.dataCorrupted) {
            this.state = CameraState.OBSTRUCTED;
            return;
        }
 
        if (this.detectedCount > 0 && averageConfidence() < DEGRADED_THRESHOLD) {
            this.state = CameraState.DEGRADED;
            return;
        }
 
        this.state = CameraState.OPERATIONAL;
        assert this.state != null : "state must not be null after updateState()";
    }
 
    /**
     * Marks this camera unit as hardware-failed.
     * Terminal state — excluded from all votes and triggers driver alert.
     */
    public void markFailed() {
        assert this.state != CameraState.FAILED : "markFailed() called on already-failed camera";
        this.state = CameraState.FAILED;
        this.dataCorrupted = true;
        this.detectedCount = 0;
        assert this.state == CameraState.FAILED : "state must be FAILED after markFailed()";
    }
 
    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------
 
    public int getSensorId()                    { return sensorId; }
    public long getTimestampMs()                { return timestampMs; }
    public CameraState getState()               { return state; }
    public LightCondition getLightCondition()   { return lightCondition; }
    public WeatherCondition getWeatherCondition() { return weatherCondition; }
    public int getDetectedCount()               { return detectedCount; }
    public boolean isDataCorrupted()            { return dataCorrupted; }

    /**
     * Returns true if this camera's output should be included in the 2oo3 vote.
     * Obstructed and Failed cameras are excluded per NF2201/NF2202.
     */
    public boolean isVotingEligible() {
        return this.state == CameraState.OPERATIONAL
            || this.state == CameraState.DEGRADED;
    }
 
    /**
     * Returns the detected object at the given index.
     * Returns NONE if index is out of range.
     */
    public ObjectType getDetectedObject(final int index) {
        if (index < 0 || index >= detectedCount) { return ObjectType.NONE; }
        return detectedObjects[index];
    }
 
    /**
     * Returns the confidence score for the detected object at the given index.
     * Returns 0.0 if index is out of range.
     */
    public double getConfidenceScore(final int index) {
        if (index < 0 || index >= detectedCount) { return 0.0; }
        return confidenceScores[index];
    }
 
    // -------------------------------------------------------------------------
    // Setters - called by simulator to set environmental conditions
    // -------------------------------------------------------------------------
 
    public void setLightCondition(final LightCondition lightCondition) {
        assert lightCondition != null : "lightCondition must not be null";
        this.lightCondition = lightCondition;
        assert this.lightCondition == lightCondition : "lightCondition was not set correctly";
    }
 
    public void setWeatherCondition(final WeatherCondition weatherCondition) {
        assert weatherCondition != null : "weatherCondition must not be null";
        this.weatherCondition = weatherCondition;
        assert this.weatherCondition == weatherCondition : "weatherCondition was not set correctly";
        updateState();
    }
 
    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------
 
    /** Clamps a confidence value to the valid [0.0, 1.0] range */
    private double clampConfidence(final double value) {
        assert !Double.isNaN(value) : "confidence value must not be NaN";
        assert !Double.isInfinite(value) : "confidence value must not be infinite";
        if (value < MIN_CONFIDENCE) { return MIN_CONFIDENCE; }
        if (value > MAX_CONFIDENCE) { return MAX_CONFIDENCE; }
        return value;
    }
 
    private double averageConfidence() {
        assert detectedCount >= 0 : "detectedCount must not be negative";
        if (detectedCount == 0) { return 0.0; }
        double sum = 0.0;
        for (int i = 0; i < detectedCount; i++) {
            sum += confidenceScores[i];
        }
        final double avg = sum / detectedCount;
        assert avg >= MIN_CONFIDENCE && avg <= MAX_CONFIDENCE : "average confidence out of range";
        return avg;
    }
}