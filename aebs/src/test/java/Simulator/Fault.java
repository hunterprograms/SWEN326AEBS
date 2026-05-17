package Simulator;

/**
 * Represents an injectable fault that can be applied to a simulation scenario.
 *
 * Faults are distinct from ScenarioEvents — events model realistic world
 * changes (braking, objects appearing), while faults model abnormal system
 * failures used to verify AEBS safety requirements under fault conditions.
 *
 * A Fault targets a specific sensor or subsystem and defines what type of
 * failure to inject, when to inject it, and how long it lasts.
 *
 * Faults are applied by the simulator on top of scenario data — the
 * underlying scenario JSON is unchanged, but the fault modifies what the
 * core AEBS system receives.
 *
 * Requirement traceability:
 *   NF2201 : SENSOR_FAILURE on camera units tests 2oo3 fallback
 *   NF2202 : OBSTRUCTION on camera tests fallback braking strategy
 *   NF4201 : CORRUPT_DATA tests redundancy catching data corruption
 *   FR2206 : SENSOR_FAILURE on LiDAR tests redundant component behaviour
 *   FR2207 : SENSOR_FAILURE on all LiDAR units tests AEBS self-disable
 *   FR3101 : SIGNAL_DELAY tests 50ms signal transmission requirement
 *   NF3102 : CORRUPT_DATA on braking tests deceleration parameter checks
 */
public final class Fault {

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    /**
     * The type of fault to inject.
     */
    public enum FaultType {

        /**
         * Sensor stops producing output entirely.
         * Models physical hardware failure or total dropout.
         * Maps to Camera.SensorStatus.FAILED / ScenarioEvent.Type.SENSOR_DROPOUT.
         * Requirement: NF2201, FR2206, FR2207
         */
        SENSOR_FAILURE,

        /**
         * Sensor output is replaced with corrupt or out-of-range values.
         * Models bit-flip errors, bus corruption, or malformed data.
         * Requirement: NF4201, NF3102
         */
        CORRUPT_DATA,

        /**
         * Sensor confidence is forced to zero, simulating physical obstruction.
         * Models mud splash, fog, or physical blockage of camera/lidar lens.
         * Requirement: NF2202
         */
        OBSTRUCTION,

        /**
         * Sensor output is delayed beyond the allowed transmission window.
         * Models communication bus latency or processing overload.
         * Requirement: FR3101
         */
        SIGNAL_DELAY,

        /**
         * Sensor reports a ghost object not present in the scenario.
         * Models radar multipath reflections or camera misclassification.
         * Requirement: NF2201 (HARA HF-008)
         */
        GHOST_OBJECT,

        /**
         * Sensor reports frozen/stale data — same reading repeated each tick.
         * Models a sensor that has locked up but not fully failed.
         * Requirement: NF4201
         */
        STALE_DATA
    }

    /**
     * Which sensor or subsystem this fault targets.
     */
    public enum FaultTarget {
        CAMERA_1,
        CAMERA_2,
        CAMERA_3,
        LIDAR_1,
        LIDAR_2,
        LIDAR_3,
        RADAR_1,
        RADAR_2,
        RADAR_3,
        WHEEL_SENSOR_1,
        WHEEL_SENSOR_2,
        WHEEL_SENSOR_3,
        DRIVER,
        BRAKING_SYSTEM
    }

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Unique identifier for this fault, e.g. "F-001" */
    public final String faultId;

    /** Human-readable description of what this fault models */
    public final String description;

    /** Which sensor or subsystem to inject the fault into */
    public final FaultTarget target;

    /** What type of fault to inject */
    public final FaultType type;

    /** Time in seconds from scenario start to inject the fault */
    public final double injectAtSec;

    /**
     * Duration of the fault in seconds.
     * 0.0 = permanent for the remainder of the scenario.
     */
    public final double durationSec;

    /**
     * Optional parameter for fault behaviour.
     * SIGNAL_DELAY: delay in milliseconds.
     * CORRUPT_DATA: corruption magnitude (0.0-1.0).
     * GHOST_OBJECT: bearing in degrees of ghost object.
     * Other types: unused (0.0).
     */
    public final double param;

    /** Requirement ID this fault is injected to verify, e.g. "NF2201" */
    public final String requirementId;

    // -------------------------------------------------------------------------
    // Constructor (private - use Builder)
    // -------------------------------------------------------------------------

    private Fault(final Builder b) {
        assert b.faultId != null       : "faultId must not be null";
        assert b.target != null        : "target must not be null";
        assert b.type != null          : "type must not be null";
        assert b.injectAtSec >= 0.0    : "injectAtSec must not be negative";
        assert b.durationSec >= 0.0    : "durationSec must not be negative";
        this.faultId       = b.faultId;
        this.description   = b.description;
        this.target        = b.target;
        this.type          = b.type;
        this.injectAtSec   = b.injectAtSec;
        this.durationSec   = b.durationSec;
        this.param         = b.param;
        this.requirementId = b.requirementId;
    }

    // -------------------------------------------------------------------------
    // Convenience query methods
    // -------------------------------------------------------------------------

    /**
     * Returns true if this fault is active at the given scenario time.
     * @param currentTimeSec current scenario time in seconds
     */
    public boolean isActiveAt(final double currentTimeSec) {
        assert currentTimeSec >= 0.0 : "currentTimeSec must not be negative";
        assert injectAtSec >= 0.0    : "injectAtSec must not be negative";
        if (currentTimeSec < injectAtSec) { return false; }
        if (durationSec == 0.0) { return true; } // permanent
        return currentTimeSec <= injectAtSec + durationSec;
    }

    /**
     * Returns true if this fault is permanent (never recovers).
     */
    public boolean isPermanent() {
        assert durationSec >= 0.0 : "durationSec must not be negative";
        assert !Double.isNaN(durationSec) : "durationSec must not be NaN";
        return durationSec == 0.0;
    }

    /**
     * Returns a short summary string for logging.
     */
    public String toDisplayString() {
        assert faultId != null  : "faultId must not be null";
        assert target != null   : "target must not be null";
        assert type != null     : "type must not be null";
        return "Fault[" + faultId + "] "
            + type + " on " + target
            + " at t=" + injectAtSec + "s"
            + (durationSec == 0.0 ? " (permanent)" : " for " + durationSec + "s")
            + " req=" + requirementId;
    }

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    public static final class Builder {
        private final String faultId;
        private final FaultTarget target;
        private final FaultType type;
        private final double injectAtSec;

        private String description   = "";
        private double durationSec   = 0.0;
        private double param         = 0.0;
        private String requirementId = "";

        /**
         * @param faultId      unique ID e.g. "F-001"
         * @param target       which sensor to inject into
         * @param type         what kind of fault
         * @param injectAtSec  when to inject (seconds from scenario start)
         */
        public Builder(final String faultId,
                       final FaultTarget target,
                       final FaultType type,
                       final double injectAtSec) {
            assert faultId != null    : "faultId must not be null";
            assert target != null     : "target must not be null";
            assert type != null       : "type must not be null";
            assert injectAtSec >= 0.0 : "injectAtSec must not be negative";
            this.faultId     = faultId;
            this.target      = target;
            this.type        = type;
            this.injectAtSec = injectAtSec;
        }

        /** Human-readable description of what this fault models */
        public Builder description(final String description) {
            assert description != null : "description must not be null";
            assert !description.isEmpty() : "description must not be empty";
            this.description = description;
            return this;
        }

        /**
         * Duration of the fault in seconds.
         * Default: 0.0 (permanent)
         */
        public Builder durationSec(final double seconds) {
            assert seconds >= 0.0 : "durationSec must not be negative";
            assert !Double.isNaN(seconds) : "durationSec must not be NaN";
            this.durationSec = seconds;
            return this;
        }

        /**
         * Optional numeric parameter for fault behaviour.
         * SIGNAL_DELAY: milliseconds of delay.
         * CORRUPT_DATA: corruption magnitude 0.0-1.0.
         * GHOST_OBJECT: bearing in degrees.
         */
        public Builder param(final double param) {
            assert !Double.isNaN(param) : "param must not be NaN";
            assert !Double.isInfinite(param) : "param must not be infinite";
            this.param = param;
            return this;
        }

        /** Requirement ID this fault verifies, e.g. "NF2201" */
        public Builder requirementId(final String requirementId) {
            assert requirementId != null : "requirementId must not be null";
            assert !requirementId.isEmpty() : "requirementId must not be empty";
            this.requirementId = requirementId;
            return this;
        }

        public Fault build() {
            assert faultId != null : "faultId must not be null before build()";
            assert target != null  : "target must not be null before build()";
            assert type != null    : "type must not be null before build()";
            assert injectAtSec >= 0.0 : "injectAtSec must not be negative before build()";
            return new Fault(this);
        }
    }
}