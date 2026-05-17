package Tester;

import swen326.group4.BrakingController.ControllerDecision;
import Tester.FaultInjector.FaultType;
import Tester.TestResult.TestReport;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * AEBSTestSuite — the authoritative catalogue of AEBS integration tests.
 *
 * ── Test philosophy ───────────────────────────────────────────────────────
 *
 * Each test exercises one or more safety requirements from the specification.
 * Tests are grouped into five suites reflecting the requirement families:
 *
 *   Group 1 — Happy-path baseline
 *     Validate the system behaves correctly under nominal conditions before
 *     introducing any fault. Every subsequent fault test is meaningful only
 *     because baseline passes.
 *
 *   Group 2 — Single-sensor fault tolerance (2oo3 architecture)
 *     FR2104 / FR2103 / FR2208 / NF2202:
 *     Fail exactly one sensor at a time (Lidar, Radar, Camera, Wheel).
 *     AEBS must still detect the hazard and engage braking via the
 *     remaining two sensors.
 *
 *   Group 3 — Two-sensor degradation (safety margin boundary)
 *     FR2104: Fail two sensors of the same modality.
 *     With only one eligible sensor remaining, the voter cannot reach
 *     the 2oo3 quorum. The expected outcomes are SENSOR_FAULT or
 *     a fallback path — no false AUTONOMOUS_BRAKE from a single sensor.
 *     FR2106 / FR2107: AEBS must not act on Lidar or Radar alone.
 *
 *   Group 4 — Driver interaction (FR3104)
 *     Driver brakes before AEBS: expect DRIVER_OVERRIDE, no autonomous brake.
 *     Driver never brakes under hazard: expect AUTONOMOUS_BRAKE.
 *     Driver brakes after AEBS starts: expect DRIVER_OVERRIDE to cancel.
 *
 *   Group 5 — Timing and escalation (FR-3105, FR-3106)
 *     Delayed sensor failure: system starts healthy then degrades.
 *     Intermittent sensor: sensor flickers — 2oo3 must tolerate noise.
 *
 * ── How to add a test ─────────────────────────────────────────────────────
 *
 *   new TestScenario.Builder("TC-NNN", "scenarioId", durationMs)
 *       .description("Human-readable description")
 *       .faultLidar1(FaultType.HARD_FAIL)          // inject fault
 *       .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
 *       .expectBrakingEngaged(true)
 *       .build()
 *
 * Requirement traceability is listed per test in inline comments.
 */
public final class AEBSTestSuite {

    // =========================================================================
    // Entry point
    // =========================================================================

    /**
     * Main entry point. Builds all scenarios, runs them, writes a report file,
     * and exits with code 0 (all passed) or 1 (any failed).
     */
    public static void main(final String[] args) throws IOException {

        final List<TestScenario> scenarios = buildAllScenarios();
        final AEBSTestRunner runner = new AEBSTestRunner(scenarios);
        final TestReport report = runner.runAll();

        /* Write report to file */
        final File reportFile = new File("aebs_test_report.txt");
        try (final FileWriter fw = new FileWriter(reportFile)) {
            fw.write(report.toFormattedReport());
        }
        System.err.println("\nReport written to: " + reportFile.getAbsolutePath());

        System.exit(report.failCount() == 0 ? 0 : 1);
    }

    // =========================================================================
    // Scenario catalogue
    // =========================================================================

    public static List<TestScenario> buildAllScenarios() {
        final List<TestScenario> list = new ArrayList<>();

        list.addAll(group1_HappyPath());
        list.addAll(group2_SingleSensorFault());
        list.addAll(group3_TwoSensorFault());
        list.addAll(group4_DriverInteraction());
        list.addAll(group5_TimingAndEscalation());

        return list;
    }

    // =========================================================================
    // Group 1 — Happy-path baseline
    // =========================================================================

    private static List<TestScenario> group1_HappyPath() {
        final List<TestScenario> g = new ArrayList<>();

        /*
         * TC-001: Nominal run — approaching vehicle, all sensors healthy.
         *
         * World: world1 (SC-001, Highway Driving)
         *   - Radar/Lidar both detect object approaching at -30 km/h from 80m.
         *   - TTC starts ~9.6s (clear), crosses alert at ~3s, brake at ~1.5s.
         *   - Driver does not brake (NONE ticks until t=1000ms).
         *
         * Expected: DRIVER_ALERT must appear before AUTONOMOUS_BRAKE.
         *           Braking must engage.
         *
         * Requirements: FR3101 (50ms cycle), FR2103/FR2106/FR2107 (sensor fusion),
         *               TTC thresholds.
         */
        g.add(new TestScenario.Builder("TC-001", "world1", 3000)
            .description("Nominal: all sensors healthy, approaching vehicle, expect brake")
            .expectDecisionAtLeastOnce(ControllerDecision.DRIVER_ALERT)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-002: No hazard — objects detected but at safe distance/bearing.
         *
         * World: a world where all objects are >100m away or off-axis bearing.
         * Replace "safe_world" with your no-hazard scenario folder name.
         *
         * Expected: only CLEAR decisions observed, never AUTONOMOUS_BRAKE.
         *
         * Requirements: FR3101 (controller produces decisions at all),
         *               no false positive braking.
         */
        g.add(new TestScenario.Builder("TC-002", "safe_world", 2000)
            .description("Nominal: no hazard present, controller stays CLEAR")
            .expectDecisionAtLeastOnce(ControllerDecision.CLEAR)
            .forbidDecision(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectNoBraking(true)
            .build());

        return g;
    }

    // =========================================================================
    // Group 2 — Single-sensor fault tolerance (FR2104, FR2208, NF2202)
    // =========================================================================

    private static List<TestScenario> group2_SingleSensorFault() {
        final List<TestScenario> g = new ArrayList<>();

        /*
         * TC-101: Lidar1 hard-fail — 2oo3 must brake using Lidar2 + Lidar3.
         *
         * FR2104: FAILED sensor excluded from vote.
         * 2oo3 architecture: remaining two Lidars agree → trusted vote → brake.
         */
        g.add(new TestScenario.Builder("TC-101", "world1", 3000)
            .description("FR2104: Lidar1 HARD_FAIL — 2oo3 must still engage brake")
            .faultLidar1(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-102: Lidar2 hard-fail — same as TC-101 but different sensor instance.
         * Verifies the fault tolerance is not tied to a specific sensor index.
         */
        g.add(new TestScenario.Builder("TC-102", "world1", 3000)
            .description("FR2104: Lidar2 HARD_FAIL — 2oo3 must still engage brake")
            .faultLidar2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-103: Radar1 hard-fail — 2oo3 must brake using Radar2 + Radar3.
         *
         * FR2107: AEBS requires radar agreement — one failed radar is tolerated,
         * two remaining reach quorum.
         */
        g.add(new TestScenario.Builder("TC-103", "world1", 3000)
            .description("FR2107: Radar1 HARD_FAIL — 2oo3 must still engage brake")
            .faultRadar1(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-104: Radar3 hard-fail.
         * Verifies the fault-tolerance path on the third radar instance.
         */
        g.add(new TestScenario.Builder("TC-104", "world1", 3000)
            .description("FR2107: Radar3 HARD_FAIL — 2oo3 must still engage brake")
            .faultRadar3(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-105: Camera1 hard-fail — CameraVoter 2oo3 uses Camera2 + Camera3.
         *
         * NF2202: FAILED camera excluded; two remaining reach quorum.
         * Camera vote is not the primary ranging source (Radar/Lidar are),
         * so braking should still engage on the ranging sensor consensus.
         */
        g.add(new TestScenario.Builder("TC-105", "world1", 3000)
            .description("NF2202: Camera1 HARD_FAIL — ranging sensors still brake")
            .faultCamera1(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-106: WheelSensor1 hard-fail — WheelVoter 2oo3 uses WS2 + WS3.
         *
         * FR2208: single frozen/erroneous wheel sensor must not affect decision.
         * Brake confirmation (FR-3108) uses WheelVoter output — two remaining
         * sensors still provide a trusted speed reading.
         */
        g.add(new TestScenario.Builder("TC-106", "world1", 3000)
            .description("FR2208: WheelSensor1 HARD_FAIL — 2oo3 wheel vote still works")
            .faultWheel1(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-107: Lidar1 degraded — confidence drops below 0.5, DEGRADED status.
         *
         * FR2103: controller should shift weight to Radar when Lidar is DEGRADED.
         * Vote is still reached (two Lidars at lower confidence + one degraded).
         * Brake must still engage because Radar voter provides trusted distance.
         */
        g.add(new TestScenario.Builder("TC-107", "world1", 3000)
            .description("FR2103: Lidar1 DEGRADED — controller uses Radar, brake still engages")
            .faultLidar1(FaultType.DEGRADED)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-108: Radar1 degraded — confidence drops, controller increases Lidar weight.
         *
         * FR2103 (inverse): DEGRADED radar shifts weight to Lidar.
         */
        g.add(new TestScenario.Builder("TC-108", "world1", 3000)
            .description("FR2103: Radar1 DEGRADED — controller uses Lidar, brake still engages")
            .faultRadar1(FaultType.DEGRADED)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-109: Lidar1 frozen reading — sensor reports stuck distance, rest healthy.
         *
         * FR4203 / 2oo3: a frozen sensor produces a reading that will diverge from
         * the other two as the vehicle approaches. The RadarLidarVoter median
         * distance check should exclude the outlier, allowing the two consistent
         * sensors to reach quorum and brake correctly.
         */
        g.add(new TestScenario.Builder("TC-109", "world1", 3000)
            .description("FR4203: Lidar1 FROZEN — divergent reading excluded, 2oo3 brakes")
            .faultLidar1(FaultType.FROZEN_READING)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        return g;
    }

    // =========================================================================
    // Group 3 — Two-sensor fault (quorum loss, no false action)
    // =========================================================================

    private static List<TestScenario> group3_TwoSensorFault() {
        final List<TestScenario> g = new ArrayList<>();

        /*
         * TC-201: Two Lidars hard-fail — quorum impossible, SENSOR_FAULT.
         *
         * FR2104 / FR2106: with only one eligible Lidar, the voter cannot trust
         * the result. Combined with Radar the controller should fall through to
         * Radar-only. FR2106 states AEBS cannot act on Lidar alone — here it acts
         * on Radar alone (which FR2107 also forbids). So:
         *   - If Radar is healthy: Radar alone is not trusted (FR2107) → SENSOR_FAULT.
         *   - SENSOR_FAULT must be raised.
         *   - AUTONOMOUS_BRAKE must NOT fire from a single ranging modality.
         *
         * Note: If your scenario is close enough that camera consensus is also
         * triggering a brake, adjust this test expectation or use a scenario with
         * no camera object detection.
         */
        g.add(new TestScenario.Builder("TC-201", "world1", 3000)
            .description("FR2104/FR2106: Lidar1+2 HARD_FAIL — quorum lost, SENSOR_FAULT expected")
            .faultLidar1(FaultType.HARD_FAIL)
            .faultLidar2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.SENSOR_FAULT)
            .forbidDecision(ControllerDecision.AUTONOMOUS_BRAKE)
            .build());

        /*
         * TC-202: Two Radars hard-fail — quorum impossible on radar side.
         *
         * FR2107: with only one eligible Radar, AEBS cannot act on radar alone.
         * Result should be SENSOR_FAULT; no autonomous brake.
         */
        g.add(new TestScenario.Builder("TC-202", "world1", 3000)
            .description("FR2107: Radar1+2 HARD_FAIL — quorum lost, SENSOR_FAULT expected")
            .faultRadar1(FaultType.HARD_FAIL)
            .faultRadar2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.SENSOR_FAULT)
            .forbidDecision(ControllerDecision.AUTONOMOUS_BRAKE)
            .build());

        /*
         * TC-203: Two Cameras hard-fail — CameraVoter fallback active.
         *
         * NF2202: fewer than 2 cameras eligible; INSUFFICIENT_CAMERAS fallback fires.
         * Camera fault should trigger SENSOR_FAULT from the camera voter path.
         * Ranging sensors (Radar/Lidar) remain healthy — braking may still engage
         * from ranging consensus if TTC is critical. This test checks the camera
         * fault is surfaced (SENSOR_FAULT observed) but does not forbid braking
         * (ranging sensors are still healthy and should still protect the vehicle).
         */
        g.add(new TestScenario.Builder("TC-203", "world1", 3000)
            .description("NF2202: Camera1+2 HARD_FAIL — insufficient cameras, SENSOR_FAULT raised")
            .faultCamera1(FaultType.HARD_FAIL)
            .faultCamera2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.SENSOR_FAULT)
            .build());

        /*
         * TC-204: Two WheelSensors hard-fail — WheelVoter loses quorum.
         *
         * FR2208: with only one wheel sensor set, speed cannot be trusted.
         * The controller should be unable to confirm braking deceleration (FR-3108)
         * and escalation should occur.
         */
        g.add(new TestScenario.Builder("TC-204", "world1", 4000)
            .description("FR2208: WheelSensor1+2 HARD_FAIL — speed unconfirmed, escalation expected")
            .faultWheel1(FaultType.HARD_FAIL)
            .faultWheel2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.ESCALATION)
            .build());

        /*
         * TC-205: All three Lidars hard-fail — Radar still healthy, should SENSOR_FAULT.
         *
         * FR2106: no Lidar data; Radar-only is not enough per FR2106+FR2107.
         * SENSOR_FAULT must be raised. AUTONOMOUS_BRAKE must not fire.
         */
        g.add(new TestScenario.Builder("TC-205", "world1", 3000)
            .description("FR2106: All Lidars HARD_FAIL — radar alone insufficient, SENSOR_FAULT")
            .faultLidar1(FaultType.HARD_FAIL)
            .faultLidar2(FaultType.HARD_FAIL)
            .faultLidar3(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.SENSOR_FAULT)
            .forbidDecision(ControllerDecision.AUTONOMOUS_BRAKE)
            .build());

        return g;
    }

    // =========================================================================
    // Group 4 — Driver interaction (FR3104, FR3103)
    // =========================================================================

    private static List<TestScenario> group4_DriverInteraction() {
        final List<TestScenario> g = new ArrayList<>();

        /*
         * TC-301: Driver brakes throughout — AEBS must yield (FR3104).
         *
         * FaultyDriver(FROZEN_READING) always returns Action.BRAKE.
         * FR3104: manual braking detected → controller yields all authority.
         * Expected: DRIVER_OVERRIDE observed; AUTONOMOUS_BRAKE never fires.
         */
        g.add(new TestScenario.Builder("TC-301", "world1", 3000)
            .description("FR3104: Driver always braking — AEBS must yield (DRIVER_OVERRIDE)")
            .faultDriver(FaultType.FROZEN_READING)           // driver always brakes
            .expectDecisionAtLeastOnce(ControllerDecision.DRIVER_OVERRIDE)
            .forbidDecision(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectNoBraking(true)
            .build());

        /*
         * TC-302: Driver never brakes under hazard — AEBS must intervene.
         *
         * FaultyDriver(SILENT) always returns Action.NONE.
         * With no manual braking detected and TTC falling below threshold,
         * AEBS must issue AUTONOMOUS_BRAKE.
         */
        g.add(new TestScenario.Builder("TC-302", "world1", 3000)
            .description("HF-002: Inattentive driver — AEBS must intervene autonomously")
            .faultDriver(FaultType.SILENT)                   // driver never brakes
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-303: Driver offline (HARD_FAIL) — treated as no manual braking.
         *
         * If the driver sensor itself fails, the controller should treat it as
         * "no manual brake input" and still proceed with autonomous intervention
         * when TTC is critical. AUTONOMOUS_BRAKE must fire.
         */
        g.add(new TestScenario.Builder("TC-303", "world1", 3000)
            .description("HF-001: Driver sensor FAILED — AEBS acts autonomously on hazard")
            .faultDriver(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        return g;
    }

    // =========================================================================
    // Group 5 — Timing, delayed faults, intermittent sensors, escalation
    // =========================================================================

    private static List<TestScenario> group5_TimingAndEscalation() {
        final List<TestScenario> g = new ArrayList<>();

        /*
         * TC-401: Lidar1 fails after 5 ticks (DELAYED_FAIL).
         *
         * System starts fully healthy, Lidar1 drops out mid-scenario.
         * Remaining Lidar2 + Lidar3 maintain quorum → brake must still engage.
         * FR2104: delayed failure handled the same as immediate failure.
         */
        g.add(new TestScenario.Builder("TC-401", "world1", 3500)
            .description("FR2104: Lidar1 delayed-fail at tick 5 — brake still engages")
            .faultLidar1(FaultType.DELAYED_FAIL)
            .delayLidar1(5)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-402: Radar1 fails after 3 ticks (DELAYED_FAIL).
         *
         * Similar to TC-401 but on the Radar modality.
         */
        g.add(new TestScenario.Builder("TC-402", "world1", 3500)
            .description("FR2107: Radar1 delayed-fail at tick 3 — brake still engages")
            .faultRadar1(FaultType.DELAYED_FAIL)
            .delayRadar1(3)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-403: Lidar1 intermittent (alternates null/valid every tick).
         *
         * An intermittent sensor sometimes contributes valid data and sometimes
         * returns null. The 2oo3 voter must handle this gracefully:
         *   - On ticks where Lidar1 is null: vote uses Lidar2 + Lidar3 (still trusted).
         *   - On ticks where Lidar1 returns a value: it may join the majority.
         * Either way, brake must engage.
         */
        g.add(new TestScenario.Builder("TC-403", "world1", 3000)
            .description("FR2104: Lidar1 intermittent — 2oo3 tolerates flicker, brakes")
            .faultLidar1(FaultType.INTERMITTENT)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-404: All sensors healthy, all braking retries fail — escalation fires.
         *
         * This test uses a scenario where the obstacle distance stays constant
         * (no deceleration feedback from wheel sensors confirms adequacy), so
         * the controller exhausts MAX_BRAKE_ATTEMPTS and escalates.
         *
         * Replace "no_decel_world" with a scenario where RPM does not drop
         * despite braking being issued (e.g., wheel sensor data is flat).
         *
         * FR-3105: escalation sent via two channels after max attempts.
         * FR-3106: escalation confirmed within ESCALATION_CONFIRM_TIMEOUT_MS.
         */
        g.add(new TestScenario.Builder("TC-404", "no_decel_world", 5000)
            .description("FR-3105: Brake retries exhausted — ESCALATION must fire")
            .expectDecisionAtLeastOnce(ControllerDecision.ESCALATION)
            .build());

        /*
         * TC-405: Camera1 fails after 10 ticks — camera voter raises alert mid-run.
         *
         * The camera voter will shift from CONSENSUS to INSUFFICIENT_CAMERAS.
         * SENSOR_FAULT must be raised mid-run. Ranging sensors remain healthy,
         * so braking can still engage.
         */
        g.add(new TestScenario.Builder("TC-405", "world1", 3500)
            .description("NF2202: Camera1 delayed-fail mid-run — SENSOR_FAULT raised, brake still fires")
            .faultCamera1(FaultType.DELAYED_FAIL)
            .delayCamera1(10)
            .expectDecisionAtLeastOnce(ControllerDecision.SENSOR_FAULT)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-406: Multiple simultaneous faults — Lidar1 + Radar2 both hard-fail.
         *
         * Cross-modality fault: one Lidar and one Radar fail simultaneously.
         * Lidar2 + Lidar3 still reach quorum. Radar1 + Radar3 still reach quorum.
         * Both voters remain trusted → AUTONOMOUS_BRAKE must still fire.
         *
         * This tests the independence of the Radar voter and Lidar voter —
         * a fault in one modality must not contaminate the other.
         */
        g.add(new TestScenario.Builder("TC-406", "world1", 3000)
            .description("FR2104/FR2107: Lidar1+Radar2 HARD_FAIL — cross-modality, brake still fires")
            .faultLidar1(FaultType.HARD_FAIL)
            .faultRadar2(FaultType.HARD_FAIL)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        /*
         * TC-407: WheelSensor1 frozen — frozen sentinel detected, excluded from vote.
         *
         * FR4203: frozen sensor (-2 sentinel) must be excluded from wheel vote.
         * With WS2 + WS3 still healthy, wheel vote reaches quorum.
         * Brake confirmation (FR-3108) and speed computation proceed normally.
         */
        g.add(new TestScenario.Builder("TC-407", "world1", 3000)
            .description("FR4203: WheelSensor1 FROZEN — sentinel excluded, 2oo3 wheel vote holds")
            .faultWheel1(FaultType.FROZEN_READING)
            .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
            .expectBrakingEngaged(true)
            .build());

        return g;
    }
}
