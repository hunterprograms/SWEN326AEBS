package swen326.group4.Sensors;

/**
 * The '2 out of 3' architecture for the camera system.
 *
 * Compares the input of three different camera sensors and
 * produces a trusted CameraReading result for the AEBS
 *
 * If fewer than 2 cameras are eligible to vote, the system enters a
 * conservative fallback state and alerts the driver.
 */

public class CameraVoter{
    /** Number of camera units in the 2oo3 architecture */
    public static final int CAMERA_COUNT = 3;

    /** Minimum number of eligible cameras required to produce a trusted vote */
    public static final int MIN_VOTING_CAMERAS = 2;

    /**
     * Result of the voting process.
     * Consumed by AEBS decision logic and driver interface.
     */
    public enum VoteResult {
        /** 2+ cameras agree - trusted classification available */
        CONSENSUS,
        /** 2+ cameras eligible but disagree on classification */
        DISAGREEMENT,
        /** Fewer than 2 cameras eligible - NF2202 fallback active */
        INSUFFICIENT_CAMERAS,
        /** All cameras failed - safe stop required */
        TOTAL_FAILURE
    }

    /** The three camera units - fixed size, no dynamic allocation */
    private final Camera[] cameras;

    /** Most recent vote result */
    private VoteResult lastVoteResult;

    /** Most recent agreed object type from consensus vote */
    private Camera.ObjectType consensusObject;

    /** Average confidence of the cameras that reached consensus */
    private double consensusConfidence;

    /** Number of cameras that were eligible in the last vote */
    private int eligibleCount;

    /**
     * Constructs the voter with exactly 3 camera units.
     * @param cam1 Camera with sensorId 1
     * @param cam2 Camera with sensorId 2
     * @param cam3 Camera with sensorId 3
     */
    public CameraVoter(final Camera cam1, final Camera cam2, final Camera cam3) {
        this.cameras = new Camera[CAMERA_COUNT];
        this.cameras[0] = cam1;
        this.cameras[1] = cam2;
        this.cameras[2] = cam3;
        this.lastVoteResult = VoteResult.INSUFFICIENT_CAMERAS;
        this.consensusObject = Camera.ObjectType.NONE;
        this.consensusConfidence = 0.0;
        this.eligibleCount = 0;
    }

    /**
     * Runs the 2oo3 vote across all camera units for the most dangerous
     * object type detected (highest priority classification).
     *
     * Called by AEBS decision logic after each 50ms camera update cycle.
     *
     * Vote process:
     *   1. Exclude OBSTRUCTED and FAILED cameras (NF2202, NF4201)
     *   2. If fewer than 2 eligible, return INSUFFICIENT_CAMERAS
     *   3. Find the highest-priority object type seen by each eligible camera
     *   4. If 2+ cameras agree on that type, return CONSENSUS
     *   5. Otherwise return DISAGREEMENT - treated conservatively by decision logic
     *
     * @return the VoteResult for this cycle
     */
    public VoteResult vote() {
        // Step 1: count eligible cameras
        eligibleCount = 0;
        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (cameras[i].isVotingEligible()) {
                eligibleCount++;
            }
        }

        // Step 2: check for total failure
        if (eligibleCount == 0) {
            lastVoteResult = VoteResult.TOTAL_FAILURE;
            consensusObject = Camera.ObjectType.NONE;
            consensusConfidence = 0.0;
            return lastVoteResult;
        }

        // Step 3: check for insufficient cameras - NF2202 fallback
        if (eligibleCount < MIN_VOTING_CAMERAS) {
            lastVoteResult = VoteResult.INSUFFICIENT_CAMERAS;
            consensusObject = Camera.ObjectType.NONE;
            consensusConfidence = 0.0;
            return lastVoteResult;
        }

        // Step 4: get the highest-priority object from each eligible camera
        final Camera.ObjectType[] topObjects = new Camera.ObjectType[CAMERA_COUNT];
        final double[] topConfidences = new double[CAMERA_COUNT];

        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (cameras[i].isVotingEligible()) {
                topObjects[i] = getHighestPriorityObject(cameras[i]);
                topConfidences[i] = getHighestPriorityConfidence(cameras[i]);
            } else {
                topObjects[i] = Camera.ObjectType.NONE;
                topConfidences[i] = 0.0;
            }
        }

        // Step 5: check for 2oo3 consensus
        final Camera.ObjectType agreed = findConsensus(topObjects);

        if (agreed != null) {
            lastVoteResult = VoteResult.CONSENSUS;
            consensusObject = agreed;
            consensusConfidence = averageConfidenceForType(topObjects, topConfidences, agreed);
        } else {
            // Disagreement - decision logic treats this conservatively
            lastVoteResult = VoteResult.DISAGREEMENT;
            consensusObject = Camera.ObjectType.UNKNOWN;
            consensusConfidence = 0.0;
        }

        return lastVoteResult;
    }

    public VoteResult getLastVoteResult()           { return lastVoteResult; }
    public Camera.ObjectType getConsensusObject()   { return consensusObject; }
    public double getConsensusConfidence()          { return consensusConfidence; }
    public int getEligibleCount()                   { return eligibleCount; }

    /**
     * Returns true if the vote produced a trusted result that decision logic
     * can act on. DISAGREEMENT is not trusted - treated as UNKNOWN.
     */
    public boolean isTrusted() {
        return lastVoteResult == VoteResult.CONSENSUS;
    }

    /**
     * Returns true if the fallback braking strategy should be applied.
     * Per NF2202: applies when fewer than 2 cameras are eligible.
     */
    public boolean isFallbackActive() {
        return lastVoteResult == VoteResult.INSUFFICIENT_CAMERAS
                || lastVoteResult == VoteResult.TOTAL_FAILURE;
    }

    /**
     * Returns true if the driver should be alerted to a camera system issue.
     * Covers all non-consensus outcomes.
     */
    public boolean requiresDriverAlert() {
        return lastVoteResult != VoteResult.CONSENSUS;
    }

    /**
     * Object priority order for voting - higher index = higher priority.
     * Pedestrians and cyclists are highest priority per HARA 2201.
     */
    private static final Camera.ObjectType[] PRIORITY_ORDER = {
            Camera.ObjectType.NONE,
            Camera.ObjectType.UNKNOWN,
            Camera.ObjectType.STATIONARY_OBJECT,
            Camera.ObjectType.VEHICLE,
            Camera.ObjectType.CYCLIST,
            Camera.ObjectType.PEDESTRIAN
    };

    /**
     * Returns the highest-priority object type detected by the given camera.
     * Priority is defined by PRIORITY_ORDER - pedestrians rank highest.
     */
    private Camera.ObjectType getHighestPriorityObject(final Camera camera) {
        Camera.ObjectType highest = Camera.ObjectType.NONE;
        int highestRank = 0;

        for (int i = 0; i < camera.getDetectedCount(); i++) {
            final Camera.ObjectType obj = camera.getDetectedObject(i);
            final int rank = getPriorityRank(obj);
            if (rank > highestRank) {
                highestRank = rank;
                highest = obj;
            }
        }
        return highest;
    }

    /**
     * Returns the confidence score for the highest-priority object in this camera.
     */
    private double getHighestPriorityConfidence(final Camera camera) {
        Camera.ObjectType highest = Camera.ObjectType.NONE;
        int highestRank = 0;
        double highestConf = 0.0;

        for (int i = 0; i < camera.getDetectedCount(); i++) {
            final Camera.ObjectType obj = camera.getDetectedObject(i);
            final int rank = getPriorityRank(obj);
            if (rank > highestRank) {
                highestRank = rank;
                highest = obj;
                highestConf = camera.getConfidenceScore(i);
            }
        }
        return highestConf;
    }

    /**
     * Returns the priority rank of an object type.
     * Higher rank = higher priority for AEBS intervention.
     */
    private int getPriorityRank(final Camera.ObjectType type) {
        for (int i = 0; i < PRIORITY_ORDER.length; i++) {
            if (PRIORITY_ORDER[i] == type) { return i; }
        }
        return 0;
    }

    /**
     * Checks if 2 or more cameras agree on the same object type.
     * Returns the agreed type if consensus is reached, null otherwise.
     */
    private Camera.ObjectType findConsensus(final Camera.ObjectType[] topObjects) {
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
     * Computes the average confidence score among cameras that reported
     * the given object type.
     */
    private double averageConfidenceForType(final Camera.ObjectType[] topObjects,
                                            final double[] topConfidences,
                                            final Camera.ObjectType type) {
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < CAMERA_COUNT; i++) {
            if (topObjects[i] == type) {
                sum += topConfidences[i];
                count++;
            }
        }
        return (count == 0) ? 0.0 : sum / count;
    }

}