package swen326.group4.Sensors;

/**
 * Immutable data transfer object representing the output of a CameraVoter cycle.
 *
 * This is the single object passed upward from the Camera subsystem to the
 * AEBS decision logic and sensor fusion layer. It carries the voted
 * classification result, confidence, and system state so that upstream
 * components never need to interact directly with individual Camera units.
 *
 * Immutability is intentional - once a reading is produced it must not
 * be altered in transit, preventing silent data corruption (NF4201).
 */
public final class CameraReading {
    /** Timestamp of the camera update cycle this reading came from */
    private final long timestampMs;

    /** Result of the 2oo3 vote that produced this reading */
    private final CameraVoter.VoteResult voteResult;

    /** Agreed object type from consensus vote, or UNKNOWN if no consensus */
    private final Camera.ObjectType detectedObject;

    /** Average confidence of cameras that agreed on detectedObject (0.0 to 1.0) */
    private final double confidence;

    /** Number of cameras that were eligible to vote in this cycle */
    private final int eligibleCameraCount;

    /**
     * True if the NF2202 fallback braking strategy should be applied.
     * Set when fewer than 2 cameras are eligible.
     */
    private final boolean fallbackActive;

    /**
     * True if the driver should be alerted to a camera system issue.
     * Set on any non-consensus outcome.
     */
    private final boolean driverAlertRequired;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     *
     * Constructs a CameraReading from the output of a completed CameraVoter cycle.
     * Called externally after each vote() call to snapshot the voter's current state.
     *
     *
     * @param timestampMs         timestamp of this reading
     * @param voteResult          result of the 2oo3 vote
     * @param detectedObject      agreed object type (UNKNOWN if no consensus)
     * @param confidence          average confidence of agreeing cameras
     * @param eligibleCameraCount number of cameras eligible to vote
     * @param fallbackActive      true if NF2202 fallback braking applies
     * @param driverAlertRequired true if driver should be notified of camera issue
     */
    public CameraReading(final long timestampMs,
                         final CameraVoter.VoteResult voteResult,
                         final Camera.ObjectType detectedObject,
                         final double confidence,
                         final int eligibleCameraCount,
                         final boolean fallbackActive,
                         final boolean driverAlertRequired) {
        this.timestampMs = timestampMs;
        this.voteResult = voteResult;
        this.detectedObject = detectedObject;
        this.confidence = confidence;
        this.eligibleCameraCount = eligibleCameraCount;
        this.fallbackActive = fallbackActive;
        this.driverAlertRequired = driverAlertRequired;
    }

    /**
     * Produces a CameraReading from a CameraVoter that has just completed vote().
     * This is the intended construction path - keeps CameraVoter state encapsulated.
     *
     * Usage:
     *   voter.vote();
     *   CameraReading reading = CameraReading.fromVoter(timestampMs, voter);
     *
     * @param timestampMs timestamp of the current update cycle
     * @param voter       the CameraVoter after vote() has been called
     * @return a new immutable CameraReading
     */
    public static CameraReading fromVoter(final long timestampMs,
                                          final CameraVoter voter) {
        return new CameraReading(
                timestampMs,
                voter.getLastVoteResult(),
                voter.getConsensusObject(),
                voter.getConsensusConfidence(),
                voter.getEligibleCount(),
                voter.isFallbackActive(),
                voter.requiresDriverAlert()
        );
    }

    public long getTimestampMs()                    { return timestampMs; }
    public CameraVoter.VoteResult getVoteResult()   { return voteResult; }
    public Camera.ObjectType getDetectedObject()    { return detectedObject; }
    public double getConfidence()                   { return confidence; }
    public int getEligibleCameraCount()             { return eligibleCameraCount; }
    public boolean isFallbackActive()               { return fallbackActive; }
    public boolean isDriverAlertRequired()          { return driverAlertRequired; }

    /**
     * Returns true if this reading contains a trusted classification.
     * Only true when vote reached CONSENSUS.
     */
    public boolean isTrusted() {
        return voteResult == CameraVoter.VoteResult.CONSENSUS;
    }

    /**
     * Returns true if a human or cyclist was the consensus detection.
     * Highest-priority intervention cases per HARA 2201.
     */
    public boolean isVulnerableRoadUser() {
        return detectedObject == Camera.ObjectType.PEDESTRIAN
                || detectedObject == Camera.ObjectType.CYCLIST;
    }

    /**
     * Returns true if any obstacle was detected with consensus.
     * Used by decision logic to determine if braking evaluation is needed.
     */
    public boolean isObstacleDetected() {
        return isTrusted()
                && detectedObject != Camera.ObjectType.NONE;
    }

    /**
     * Returns a short string summary for logging and driver interface display.
     * Kept simple - no dynamic allocation beyond what the runtime provides.
     */
    public String toDisplayString() {
        return "CAM t=" + timestampMs
                + " vote=" + voteResult
                + " obj=" + detectedObject
                + " conf=" + confidence
                + " eligible=" + eligibleCameraCount
                + " fallback=" + fallbackActive
                + " alert=" + driverAlertRequired;
    }
}