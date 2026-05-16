package swen326.group4.Sensors.Camera;

/**
 * Implements 2oo3 (2-out-of-3) voting across three Camera sensor units.
 *
 * Collects the latest CameraReading from Camera(1), Camera(2), Camera(3),
 * excludes any that are not voting-eligible (OBSTRUCTED or FAILED),
 * and produces a CameraVoteResult for the AEBS control layer.
 *
 * Called every 50ms in sync with the Camera update cycle.
 *
 * Requirement traceability:
 *   NF2201 : 2oo3 voting for correct object classification
 *   NF2202 : Fallback when fewer than 2 cameras are eligible
 *   NF4201 : Corrupted/failed cameras excluded from vote
 */
public class CameraVoter {

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    /**
     * Result of the 2oo3 voting process.
     * Consumed by AEBS control and driver interface.
     */
    public enum VoteResult {
        /** 2+ cameras agree on a classification - trusted result */
        CONSENSUS,
        /** 2+ cameras eligible but disagree on classification */
        DISAGREEMENT,
        /** Fewer than 2 cameras eligible - NF2202 fallback active */
        INSUFFICIENT_CAMERAS,
        /** All cameras failed - safe stop required */
        TOTAL_FAILURE
    }

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /** Number of camera units in the 2oo3 architecture */
    public static final int CAMERA_COUNT = 3;

    /** Minimum eligible cameras required for a trusted vote */
    public static final int MIN_VOTING_CAMERAS = 2;

    /**
     * Object priority order - higher index = higher priority.
     * Pedestrians highest per HARA 2201.
     */
    private static final Camera.ObjectType[] PRIORITY_ORDER = {
        Camera.ObjectType.NONE,
        Camera.ObjectType.UNKNOWN,
        Camera.ObjectType.STATIONARY_OBJECT,
        Camera.ObjectType.VEHICLE,
        Camera.ObjectType.CYCLIST,
        Camera.ObjectType.PEDESTRIAN
    };

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** The three camera units - fixed size, no dynamic allocation */
    private final Camera[] cameras;

    /** Most recent vote result */
    private VoteResult lastVoteResult;

    /** Agreed object type from the most recent consensus vote */
    private Camera.ObjectType consensusObject;

    /** Average confidence of cameras that agreed in last vote */
    private float consensusConfidence;

    /** Number of cameras eligible in the last vote */
    private int eligibleCount;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Constructs the voter with exactly 3 Camera units.
     * @param cam1 Camera with sensorId "1"
     * @param cam2 Camera with sensorId "2"
     * @param cam3 Camera with sensorId "3"
     */
    public CameraVoter(final Camera cam1, final Camera cam2, final Camera cam3) {
        assert cam1 != null : "cam1 must not be null";
        assert cam2 != null : "cam2 must not be null";
        assert cam3 != null : "cam3 must not be null";
        this.cameras = new Camera[CAMERA_COUNT];
        this.cameras[0] = cam1;
        this.cameras[1] = cam2;
        this.cameras[2] = cam3;
        this.lastVoteResult = VoteResult.INSUFFICIENT_CAMERAS;
        this.consensusObject = Camera.ObjectType.NONE;
        this.consensusConfidence = 0.0f;
        this.eligibleCount = 0;
    }

    // -------------------------------------------------------------------------
    // Core voting method - called every 50ms
    // -------------------------------------------------------------------------

    /**
     * Runs the 2oo3 vote across all camera units.
     *
     * Vote process:
     *   1. Exclude OBSTRUCTED and FAILED cameras
     *   2. If fewer than 2 eligible, return INSUFFICIENT_CAMERAS
     *   3. Get highest-priority object from each eligible camera's latest reading
     *   4. If 2+ agree, return CONSENSUS
     *   5. Otherwise return DISAGREEMENT
     *
     * @return the VoteResult for this cycle
     */
    public VoteResult vote() {
        assert cameras[0] != null : "camera 1 must not be null";
        assert cameras[1] != null : "camera 2 must not be null";
        assert cameras[2] != null : "camera 3 must not be null";

        // Step 1: count eligible cameras
        eligibleCount = 0;
        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (cameras[i].isVotingEligible()) {
                eligibleCount++;
            }
        }

        // Step 2: total failure
        if (eligibleCount == 0) {
            lastVoteResult = VoteResult.TOTAL_FAILURE;
            consensusObject = Camera.ObjectType.NONE;
            consensusConfidence = 0.0f;
            return lastVoteResult;
        }

        // Step 3: insufficient cameras - NF2202 fallback
        if (eligibleCount < MIN_VOTING_CAMERAS) {
            lastVoteResult = VoteResult.INSUFFICIENT_CAMERAS;
            consensusObject = Camera.ObjectType.NONE;
            consensusConfidence = 0.0f;
            return lastVoteResult;
        }

        // Step 4: get highest-priority object from each eligible camera
        final Camera.ObjectType[] topObjects = new Camera.ObjectType[CAMERA_COUNT];
        final float[] topConfidences = new float[CAMERA_COUNT];

        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (cameras[i].isVotingEligible()) {
                final CameraReading reading = cameras[i].getLatestReading();
                if (reading != null) {
                    topObjects[i]     = getHighestPriorityObject(reading);
                    topConfidences[i] = getHighestPriorityConfidence(reading);
                } else {
                    topObjects[i]     = Camera.ObjectType.NONE;
                    topConfidences[i] = 0.0f;
                }
            } else {
                topObjects[i]     = Camera.ObjectType.NONE;
                topConfidences[i] = 0.0f;
            }
        }

        // Step 5: check for 2oo3 consensus
        final Camera.ObjectType agreed = findConsensus(topObjects);

        if (agreed != null) {
            lastVoteResult    = VoteResult.CONSENSUS;
            consensusObject   = agreed;
            consensusConfidence = averageConfidenceForType(topObjects, topConfidences, agreed);
        } else {
            lastVoteResult    = VoteResult.DISAGREEMENT;
            consensusObject   = Camera.ObjectType.UNKNOWN;
            consensusConfidence = 0.0f;
        }

        assert lastVoteResult != null : "lastVoteResult must not be null after vote()";
        assert consensusObject != null : "consensusObject must not be null after vote()";
        return lastVoteResult;
    }

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------

    public VoteResult getLastVoteResult()         { return lastVoteResult; }
    public Camera.ObjectType getConsensusObject() { return consensusObject; }
    public float getConsensusConfidence()         { return consensusConfidence; }
    public int getEligibleCount()                 { return eligibleCount; }

    /**
     * True only on CONSENSUS — safe for AEBS control to act on.
     */
    public boolean isTrusted() {
        assert lastVoteResult != null : "lastVoteResult must not be null";
        assert consensusObject != null : "consensusObject must not be null";
        return lastVoteResult == VoteResult.CONSENSUS;
    }

    /**
     * True when NF2202 fallback braking should apply.
     * Fewer than 2 cameras eligible.
     */
    public boolean isFallbackActive() {
        assert lastVoteResult != null : "lastVoteResult must not be null";
        assert eligibleCount >= 0 : "eligibleCount must not be negative";
        return lastVoteResult == VoteResult.INSUFFICIENT_CAMERAS
            || lastVoteResult == VoteResult.TOTAL_FAILURE;
    }

    /**
     * True when driver should be alerted to a camera system issue.
     */
    public boolean requiresDriverAlert() {
        assert lastVoteResult != null : "lastVoteResult must not be null";
        assert eligibleCount >= 0 : "eligibleCount must not be negative";
        return lastVoteResult != VoteResult.CONSENSUS;
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the highest-priority ObjectType in the given reading.
     */
    private Camera.ObjectType getHighestPriorityObject(final CameraReading reading) {
        assert reading != null : "reading must not be null";
        assert reading.count() >= 0 : "reading count must not be negative";
        Camera.ObjectType highest = Camera.ObjectType.NONE;
        int highestRank = 0;
        for (int i = 0; i < reading.count(); i++) {
            final int rank = getPriorityRank(reading.detectedObjects()[i]);
            if (rank > highestRank) {
                highestRank = rank;
                highest = reading.detectedObjects()[i];
            }
        }
        return highest;
    }

    /**
     * Returns the confidence score for the highest-priority object in the reading.
     */
    private float getHighestPriorityConfidence(final CameraReading reading) {
        assert reading != null : "reading must not be null";
        assert reading.count() >= 0 : "reading count must not be negative";
        int highestRank = 0;
        float highestConf = 0.0f;
        for (int i = 0; i < reading.count(); i++) {
            final int rank = getPriorityRank(reading.detectedObjects()[i]);
            if (rank > highestRank) {
                highestRank = rank;
                highestConf = reading.confidenceScores()[i];
            }
        }
        assert highestConf >= 0.0f && highestConf <= 1.0f : "confidence must be in range";
        return highestConf;
    }

    /**
     * Returns the priority rank of an ObjectType.
     * Higher rank = higher priority for AEBS intervention.
     */
    private int getPriorityRank(final Camera.ObjectType type) {
        assert type != null : "type must not be null";
        for (int i = 0; i < PRIORITY_ORDER.length; i++) {
            if (PRIORITY_ORDER[i] == type) {
                assert i >= 0 && i < PRIORITY_ORDER.length : "rank must be within bounds";
                return i;
            }
        }
        return 0;
    }

    /**
     * Returns the agreed ObjectType if 2+ cameras report the same type.
     * Returns null if no consensus is reached.
     */
    private Camera.ObjectType findConsensus(final Camera.ObjectType[] topObjects) {
        assert topObjects != null : "topObjects must not be null";
        assert topObjects.length == CAMERA_COUNT : "topObjects length must equal CAMERA_COUNT";
        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (topObjects[i] == Camera.ObjectType.NONE) { continue; }
            int count = 0;
            for (int j = 0; j < CAMERA_COUNT; j++) {
                if (topObjects[j] == topObjects[i]) { count++; }
            }
            if (count >= MIN_VOTING_CAMERAS) { return topObjects[i]; }
        }
        return null;
    }

    /**
     * Computes the average confidence among cameras that reported the given type.
     */
    private float averageConfidenceForType(final Camera.ObjectType[] topObjects,
                                           final float[] topConfidences,
                                           final Camera.ObjectType type) {
        assert topObjects != null : "topObjects must not be null";
        assert topConfidences != null : "topConfidences must not be null";
        assert type != null : "type must not be null";
        assert topObjects.length == topConfidences.length : "arrays must be same length";
        float sum = 0.0f;
        int count = 0;
        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (topObjects[i] == type) {
                sum += topConfidences[i];
                count++;
            }
        }
        final float result = (count == 0) ? 0.0f : sum / count;
        assert result >= 0.0f && result <= 1.0f : "average confidence must be in range";
        return result;
    }
}