package Tester;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import Tester.FaultInjector.FaultType;
import swen326.group4.BrakingController.ControllerDecision;

/**
 * TestScenario — declarative definition of a single AEBS integration test.
 *
 * A TestScenario is built via its inner Builder, which lets callers specify:
 *   - which scenario data folder to load (scenarioId)
 *   - how long to run the scenario (durationMs)
 *   - what fault to inject into any of the 13 sensor instances
 *   - what ControllerDecision is expected at least once during the run
 *   - what ControllerDecision must NEVER appear during the run
 *   - whether autonomous braking is expected to activate (BrakeActuator check)
 *   - an optional human-readable description for report output
 *
 * Design goals:
 *   - Zero ambiguity: every field has a typed, named accessor.
 *   - Readable at the call site:
 *       new TestScenario.Builder("TC-001", "world1", 3000)
 *           .description("LiDAR1 hard-fail: 2oo3 should still brake")
 *           .faultLidar1(FaultType.HARD_FAIL)
 *           .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
 *           .expectBrakingEngaged(true)
 *           .build();
 *
 * Requirement traceability:
 *   FR2104  : tested by HARD_FAIL on any single Lidar/Radar + expectBraking.
 *   FR2103  : tested by DEGRADED on one sensor + expectDecisionAtLeastOnce(CLEAR)
 *             to verify no false brake.
 *   FR2208  : tested by WheelSensor faults.
 *   FR3104  : tested by FaultyDriver(FROZEN_READING) + expectDecision(DRIVER_OVERRIDE).
 *   FR-3105 : tested by all sensors OK but expectDecision(ESCALATION) to cover
 *             the exhausted-brake-attempts path.
 *   NF2202  : tested by Camera HARD_FAIL (2 cameras) + expectBraking fallback.
 */
public final class TestScenario {

    // =========================================================================
    // Sensor fault slots — one per sensor instance (3 radars, 3 lidars,
    // 3 cameras, 3 wheel sensors, 1 driver)
    // =========================================================================

    private final String         testId;
    private final String         description;
    private final String         scenarioId;
    private final int            durationMs;

    /* Camera faults */
    private final FaultType      faultCamera1;
    private final FaultType      faultCamera2;
    private final FaultType      faultCamera3;

    /* Radar faults */
    private final FaultType      faultRadar1;
    private final FaultType      faultRadar2;
    private final FaultType      faultRadar3;

    /* Lidar faults */
    private final FaultType      faultLidar1;
    private final FaultType      faultLidar2;
    private final FaultType      faultLidar3;

    /* Wheel sensor faults */
    private final FaultType      faultWheel1;
    private final FaultType      faultWheel2;
    private final FaultType      faultWheel3;

    /* Driver fault */
    private final FaultType      faultDriver;

    /* Delay ticks for DELAYED_FAIL faults (sensor-specific) */
    private final int            delayLidar1;
    private final int            delayRadar1;
    private final int            delayCamera1;

    // =========================================================================
    // Expected outcomes
    // =========================================================================

    /** Decisions that must appear at least once during the run. */
    private final List<ControllerDecision> requiredDecisions;

    /** Decisions that must NEVER appear during the run. */
    private final List<ControllerDecision> forbiddenDecisions;

    /** True if autonomous braking (BrakeActuator intensity > 0) must activate. */
    private final boolean expectBrakingEngaged;

    /** True if the BrakeActuator intensity must stay at 0 throughout. */
    private final boolean expectNoBraking;

    // =========================================================================
    // Private constructor — use Builder
    // =========================================================================

    private TestScenario(final Builder b) {
        this.testId              = b.testId;
        this.description         = b.description;
        this.scenarioId          = b.scenarioId;
        this.durationMs          = b.durationMs;

        this.faultCamera1        = b.faultCamera1;
        this.faultCamera2        = b.faultCamera2;
        this.faultCamera3        = b.faultCamera3;
        this.faultRadar1         = b.faultRadar1;
        this.faultRadar2         = b.faultRadar2;
        this.faultRadar3         = b.faultRadar3;
        this.faultLidar1         = b.faultLidar1;
        this.faultLidar2         = b.faultLidar2;
        this.faultLidar3         = b.faultLidar3;
        this.faultWheel1         = b.faultWheel1;
        this.faultWheel2         = b.faultWheel2;
        this.faultWheel3         = b.faultWheel3;
        this.faultDriver         = b.faultDriver;

        this.delayLidar1         = b.delayLidar1;
        this.delayRadar1         = b.delayRadar1;
        this.delayCamera1        = b.delayCamera1;

        this.requiredDecisions   = Collections.unmodifiableList(new ArrayList<>(b.requiredDecisions));
        this.forbiddenDecisions  = Collections.unmodifiableList(new ArrayList<>(b.forbiddenDecisions));
        this.expectBrakingEngaged= b.expectBrakingEngaged;
        this.expectNoBraking     = b.expectNoBraking;
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    public String         getTestId()                { return testId; }
    public String         getDescription()           { return description; }
    public String         getScenarioId()            { return scenarioId; }
    public int            getDurationMs()            { return durationMs; }

    public FaultType      getFaultCamera1()          { return faultCamera1; }
    public FaultType      getFaultCamera2()          { return faultCamera2; }
    public FaultType      getFaultCamera3()          { return faultCamera3; }

    public FaultType      getFaultRadar1()           { return faultRadar1; }
    public FaultType      getFaultRadar2()           { return faultRadar2; }
    public FaultType      getFaultRadar3()           { return faultRadar3; }

    public FaultType      getFaultLidar1()           { return faultLidar1; }
    public FaultType      getFaultLidar2()           { return faultLidar2; }
    public FaultType      getFaultLidar3()           { return faultLidar3; }

    public FaultType      getFaultWheel1()           { return faultWheel1; }
    public FaultType      getFaultWheel2()           { return faultWheel2; }
    public FaultType      getFaultWheel3()           { return faultWheel3; }

    public FaultType      getFaultDriver()           { return faultDriver; }

    public int            getDelayLidar1()           { return delayLidar1; }
    public int            getDelayRadar1()           { return delayRadar1; }
    public int            getDelayCamera1()          { return delayCamera1; }

    public List<ControllerDecision> getRequiredDecisions()  { return requiredDecisions; }
    public List<ControllerDecision> getForbiddenDecisions() { return forbiddenDecisions; }
    public boolean        isExpectBrakingEngaged()   { return expectBrakingEngaged; }
    public boolean        isExpectNoBraking()        { return expectNoBraking; }

    @Override
    public String toString() {
        return "[" + testId + "] " + description
            + " | scenario=" + scenarioId
            + " | duration=" + durationMs + "ms";
    }

    // =========================================================================
    // Builder
    // =========================================================================

    public static final class Builder {

        private final String testId;
        private String  description   = "(no description)";
        private final String scenarioId;
        private final int    durationMs;

        /* Default: no faults on any sensor */
        private FaultType faultCamera1 = FaultType.NONE;
        private FaultType faultCamera2 = FaultType.NONE;
        private FaultType faultCamera3 = FaultType.NONE;
        private FaultType faultRadar1  = FaultType.NONE;
        private FaultType faultRadar2  = FaultType.NONE;
        private FaultType faultRadar3  = FaultType.NONE;
        private FaultType faultLidar1  = FaultType.NONE;
        private FaultType faultLidar2  = FaultType.NONE;
        private FaultType faultLidar3  = FaultType.NONE;
        private FaultType faultWheel1  = FaultType.NONE;
        private FaultType faultWheel2  = FaultType.NONE;
        private FaultType faultWheel3  = FaultType.NONE;
        private FaultType faultDriver  = FaultType.NONE;

        private int delayLidar1  = 0;
        private int delayRadar1  = 0;
        private int delayCamera1 = 0;

        private final List<ControllerDecision> requiredDecisions  = new ArrayList<>();
        private final List<ControllerDecision> forbiddenDecisions = new ArrayList<>();
        private boolean expectBrakingEngaged = false;
        private boolean expectNoBraking      = false;

        /**
         * @param testId      unique identifier for this test, e.g. "TC-001"
         * @param scenarioId  subfolder name under scenarios/, e.g. "world1"
         * @param durationMs  how long to run the scenario in milliseconds
         */
        public Builder(final String testId, final String scenarioId, final int durationMs) {
            assert testId     != null && !testId.isEmpty()     : "testId must not be empty";
            assert scenarioId != null && !scenarioId.isEmpty() : "scenarioId must not be empty";
            assert durationMs > 0                              : "durationMs must be positive";
            this.testId     = testId;
            this.scenarioId = scenarioId;
            this.durationMs = durationMs;
        }

        public Builder description(final String desc)       { this.description = desc;   return this; }

        /* ---- Camera faults ---- */
        public Builder faultCamera1(final FaultType f)      { this.faultCamera1 = f;     return this; }
        public Builder faultCamera2(final FaultType f)      { this.faultCamera2 = f;     return this; }
        public Builder faultCamera3(final FaultType f)      { this.faultCamera3 = f;     return this; }

        /* ---- Radar faults ---- */
        public Builder faultRadar1(final FaultType f)       { this.faultRadar1 = f;      return this; }
        public Builder faultRadar2(final FaultType f)       { this.faultRadar2 = f;      return this; }
        public Builder faultRadar3(final FaultType f)       { this.faultRadar3 = f;      return this; }

        /* ---- Lidar faults ---- */
        public Builder faultLidar1(final FaultType f)       { this.faultLidar1 = f;      return this; }
        public Builder faultLidar2(final FaultType f)       { this.faultLidar2 = f;      return this; }
        public Builder faultLidar3(final FaultType f)       { this.faultLidar3 = f;      return this; }

        /* ---- Wheel sensor faults ---- */
        public Builder faultWheel1(final FaultType f)       { this.faultWheel1 = f;      return this; }
        public Builder faultWheel2(final FaultType f)       { this.faultWheel2 = f;      return this; }
        public Builder faultWheel3(final FaultType f)       { this.faultWheel3 = f;      return this; }

        /* ---- Driver fault ---- */
        public Builder faultDriver(final FaultType f)       { this.faultDriver = f;      return this; }

        /* ---- Delayed-fail configuration ---- */
        public Builder delayLidar1(final int ticks)         { this.delayLidar1 = ticks;  return this; }
        public Builder delayRadar1(final int ticks)         { this.delayRadar1 = ticks;  return this; }
        public Builder delayCamera1(final int ticks)        { this.delayCamera1 = ticks; return this; }

        /* ---- Outcome expectations ---- */

        /**
         * Assert that this ControllerDecision appears at least once during the run.
         * Multiple calls accumulate (all listed decisions must be observed).
         */
        public Builder expectDecisionAtLeastOnce(final ControllerDecision d) {
            requiredDecisions.add(d);
            return this;
        }

        /**
         * Assert that this ControllerDecision never appears during the run.
         * Multiple calls accumulate.
         */
        public Builder forbidDecision(final ControllerDecision d) {
            forbiddenDecisions.add(d);
            return this;
        }

        /**
         * Assert that the BrakeActuator intensity rises above 0 at some point,
         * confirming that autonomous braking was physically commanded.
         */
        public Builder expectBrakingEngaged(final boolean expected) {
            this.expectBrakingEngaged = expected;
            return this;
        }

        /**
         * Assert that the BrakeActuator intensity stays at 0.0 throughout the
         * entire run — no autonomous braking must be commanded.
         */
        public Builder expectNoBraking(final boolean expected) {
            this.expectNoBraking = expected;
            return this;
        }

        public TestScenario build() {
            return new TestScenario(this);
        }
    }
}
