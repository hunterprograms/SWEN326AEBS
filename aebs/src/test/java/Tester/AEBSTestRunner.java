package Tester;

import swen326.group4.BrakingController;
import swen326.group4.BrakingController.ControllerDecision;
import swen326.group4.BrakeActuator;
import swen326.group4.Car.DIDInterface;
import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;
import Tester.FaultInjector.FaultType;
import Tester.TestResult.TestReport;

import java.io.File;
import java.io.PrintStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

/**
 * AEBSTestRunner — executes a list of TestScenarios and produces a TestReport.
 *
 * For each scenario:
 *   1. Resolves the scenario data directory from the configured scenarios root.
 *   2. Constructs all 13 sensor instances, applying FaultInjector wrappers
 *      wherever the scenario declares a non-NONE fault.
 *   3. Constructs a BrakingController with stub escalation channels.
 *   4. Starts all sensors, waits for initial data, then starts the controller.
 *   5. Polls getLastDecision() every POLL_INTERVAL_MS until durationMs elapses.
 *   6. Stops all components cleanly.
 *   7. Evaluates all assertions from the TestScenario against collected data.
 *   8. Records a TestResult (PASS or FAIL with reasons).
 *
 * All console output during a scenario run is suppressed to keep test output clean.
 * The runner logs test progress to the real stderr so failures are always visible.
 *
 * ── Key design decisions ──────────────────────────────────────────────────────
 *
 *   FaultyCamera / FaultyRadarLidar / FaultyWheelSensor / FaultyDriver wrap the
 *   real sensors. For NONE faults the real sensor is started normally (file-based).
 *   For all other faults the real sensor's timer is never started, so no file
 *   access occurs — the fault wrapper provides all output.
 *
 *   This means the test suite can run without having all three copies of every
 *   scenario file present. If Camera1 is HARD_FAIL, only Camera2 and Camera3
 *   need their data files. The runner catches FileNotFoundExceptions from the
 *   real sensors gracefully — they transition to FAILED status on their own.
 *
 *   BrakeActuator state is read from the static field BrakingController.brakeActuator
 *   which is the shared actuator instance. Between tests the intensity is reset to 0
 *   to prevent bleed-over.
 *
 * ── Thread safety note ────────────────────────────────────────────────────────
 *
 *   The controller and sensors run on their own Timer threads. The main test thread
 *   sleeps for durationMs and polls decisions. This is intentionally the same
 *   execution model as AEBS.java (the production entry point), so the test suite
 *   exercises real concurrency rather than a mocked single-thread path.
 *
 * Requirement traceability:
 *   FR3101 : 50ms decision cycle validated by checking decisions are produced.
 *   FR2104 : HARD_FAIL on one sensor, expect non-zero brake decisions.
 *   FR2103 : DEGRADED sensor, expect controller still acts on remaining sensors.
 *   FR3104 : Driver override tested by FaultyDriver(FROZEN_READING).
 *   FR-3105: Escalation tested by scenario where all brake retries fail.
 */
public final class AEBSTestRunner {

    /** Root directory containing scenario subdirectories. */
    private static final String SCENARIOS_ROOT = "scenarios/";

    /** How often (ms) the runner polls getLastDecision() during a run. */
    private static final int POLL_INTERVAL_MS = 50;

    /** Warm-up period after sensors start before controller is launched (ms). */
    private static final int SENSOR_WARMUP_MS = 200;

    private final List<TestScenario> scenarios;

    /**
     * Constructs a runner with the given list of scenarios.
     * @param scenarios the ordered list of test cases to execute
     */
    public AEBSTestRunner(final List<TestScenario> scenarios) {
        assert scenarios != null : "scenarios list must not be null";
        this.scenarios = new ArrayList<>(scenarios);
    }

    // =========================================================================
    // Public entry point
    // =========================================================================

    /**
     * Runs all scenarios in order and returns a TestReport.
     *
     * Progress (test name, PASS/FAIL) is written to the real stderr so it is
     * always visible regardless of any System.out redirection.
     */
    public TestReport runAll() {
        final PrintStream realErr = System.err;
        final long suiteStart = System.currentTimeMillis();

        final List<TestResult> results = new ArrayList<>();

        realErr.println("\n╔══════════════════════════════════════════════════╗");
        realErr.println("║  AEBS Test Runner — Starting " + scenarios.size() + " test(s)          ║");
        realErr.println("╚══════════════════════════════════════════════════╝");

        for (final TestScenario scenario : scenarios) {
            realErr.println("\n  ▶ Running: " + scenario);
            final TestResult result = runScenario(scenario, realErr);
            results.add(result);
            realErr.println("  " + (result.isPassed() ? "✔ PASS" : "✘ FAIL")
                + " — " + scenario.getTestId()
                + " (" + result.getWallClockMs() + "ms)");
            if (!result.isPassed()) {
                for (final String reason : result.getFailureReasons()) {
                    realErr.println("       ✗ " + reason);
                }
            }
        }

        final long suiteEnd = System.currentTimeMillis();
        final TestReport report = new TestReport(results, suiteEnd - suiteStart);

        realErr.println("\n" + report.toFormattedReport());
        return report;
    }

    // =========================================================================
    // Single scenario execution
    // =========================================================================

    /**
     * Runs a single TestScenario, collects decisions and brake intensity, then
     * evaluates all assertions.
     */
    private TestResult runScenario(final TestScenario sc, final PrintStream log) {
        final long wallStart = System.currentTimeMillis();

        /* ── Redirect System.out to /dev/null to suppress sensor spam ─────── */
        final PrintStream originalOut = System.out;
        try {
            System.setOut(new PrintStream(new java.io.OutputStream() {
                @Override public void write(int b) {}
            }));
        } catch (Exception ignored) { /* keep original if redirect fails */ }

        /* ── Resolve scenario directory ─────────────────────────────────── */
        final String dir = resolveScenarioDir(sc.getScenarioId());
        if (dir == null) {
            System.setOut(originalOut);
            return fail(sc, wallStart,
                "Scenario directory not found for scenarioId='" + sc.getScenarioId() + "'");
        }

        /* ── Reset shared BrakeActuator between test runs ────────────────── */
        BrakingController.brakeActuator.applyBrake(0.0f);

        /* ── Build sensors ───────────────────────────────────────────────── */
        final Camera cam1 = buildCamera("1", dir, sc.getFaultCamera1(), sc.getDelayCamera1());
        final Camera cam2 = buildCamera("2", dir, sc.getFaultCamera2(), 0);
        final Camera cam3 = buildCamera("3", dir, sc.getFaultCamera3(), 0);

        final Radar rawRadar1 = new Radar("1", dir);
        final Radar rawRadar2 = new Radar("2", dir);
        final Radar rawRadar3 = new Radar("3", dir);

        final RadarLidarSensor radar1 = wrapRadarLidar(rawRadar1, sc.getFaultRadar1(),
                                                        sc.getDelayRadar1(), "Radar1");
        final RadarLidarSensor radar2 = wrapRadarLidar(rawRadar2, sc.getFaultRadar2(), 0, "Radar2");
        final RadarLidarSensor radar3 = wrapRadarLidar(rawRadar3, sc.getFaultRadar3(), 0, "Radar3");

        final Lidar rawLidar1 = new Lidar("1", dir);
        final Lidar rawLidar2 = new Lidar("2", dir);
        final Lidar rawLidar3 = new Lidar("3", dir);

        final RadarLidarSensor lidar1 = wrapRadarLidar(rawLidar1, sc.getFaultLidar1(),
                                                        sc.getDelayLidar1(), "Lidar1");
        final RadarLidarSensor lidar2 = wrapRadarLidar(rawLidar2, sc.getFaultLidar2(), 0, "Lidar2");
        final RadarLidarSensor lidar3 = wrapRadarLidar(rawLidar3, sc.getFaultLidar3(), 0, "Lidar3");

        final WheelSensor ws1 = buildWheel("1", dir, sc.getFaultWheel1(), 0);
        final WheelSensor ws2 = buildWheel("2", dir, sc.getFaultWheel2(), 0);
        final WheelSensor ws3 = buildWheel("3", dir, sc.getFaultWheel3(), 0);

        final Driver driver = buildDriver(dir, sc.getFaultDriver());

        /* ── Stub escalation channels ────────────────────────────────────── */
        final boolean[] escalated = { false };
        final BrakingController.EscalationChannel chanA = new BrakingController.EscalationChannel() {
            private boolean confirmed = false;
            @Override public void sendAlert()              { escalated[0] = true; confirmed = true; }
            @Override public boolean isDeliveryConfirmed() { return confirmed; }
            @Override public String channelName()          { return "TestChannelA"; }
        };
        final BrakingController.EscalationChannel chanB = new BrakingController.EscalationChannel() {
            private boolean confirmed = false;
            @Override public void sendAlert()              { escalated[0] = true; confirmed = true; }
            @Override public boolean isDeliveryConfirmed() { return confirmed; }
            @Override public String channelName()          { return "TestChannelB"; }
        };

        /* ── DID interface (no-op for tests) ─────────────────────────────── */
        final DIDInterface did = new DIDInterface();
        did.initialize();

        /* ── Controller construction requires Radar/Lidar to be the
               declared types, but we have wrapped them as RadarLidarSensor.
               The BrakingController constructor accepts Camera/Radar/Lidar
               concrete types. For fault-free sensors we pass the real instance.
               For wrapped sensors we need a small adapter shim. ───────────── */

        /*
         * IMPORTANT: BrakingController takes concrete Camera, Radar, Lidar types.
         * FaultyRadarLidar implements RadarLidarSensor (the common interface) but
         * is NOT a subclass of Radar or Lidar. For faulted sensors we therefore
         * use a special constructor overload that accepts RadarLidarSensor.
         *
         * The test-aware version of BrakingController below is
         * BrakingControllerTestAdapter, a minimal subclass that replaces the
         * RadarLidarVoter construction to use the already-wrapped sensors
         * passed as RadarLidarSensor rather than the raw Radar/Lidar instances.
         *
         * See BrakingControllerTestAdapter.java for details.
         */
        final BrakingController controller = new BrakingControllerTestAdapter(
            cam1, cam2, cam3,
            radar1, radar2, radar3,
            lidar1, lidar2, lidar3,
            ws1, ws2, ws3,
            driver,
            chanA, chanB,
            did
        );

        /* ── Start sensors ───────────────────────────────────────────────── */
        cam1.start(); cam2.start(); cam3.start();
        radar1.start(); radar2.start(); radar3.start();
        lidar1.start(); lidar2.start(); lidar3.start();
        ws1.start(); ws2.start(); ws3.start();
        driver.start();

        /* ── Warm-up pause ───────────────────────────────────────────────── */
        sleep(SENSOR_WARMUP_MS);

        /* ── Start controller and collect decisions ──────────────────────── */
        controller.start();

        final List<ControllerDecision> observed = new ArrayList<>();
        float maxBrakeIntensity = 0.0f;

        final long scenarioEnd = System.currentTimeMillis() + sc.getDurationMs();
        while (System.currentTimeMillis() < scenarioEnd) {
            sleep(POLL_INTERVAL_MS);
            final ControllerDecision d = controller.getLastDecision();
            if (d != null) {
                observed.add(d);
            }
            final float intensity = BrakingController.brakeActuator.getCurrentIntensity();
            if (intensity > maxBrakeIntensity) {
                maxBrakeIntensity = intensity;
            }
        }

        /* ── Shutdown ────────────────────────────────────────────────────── */
        did.end();
        controller.stop();
        driver.stop();
        ws3.stop(); ws2.stop(); ws1.stop();
        lidar3.stop(); lidar2.stop(); lidar1.stop();
        radar3.stop(); radar2.stop(); radar1.stop();
        cam3.stop(); cam2.stop(); cam1.stop();

        System.setOut(originalOut);

        /* ── Evaluate assertions ─────────────────────────────────────────── */
        final List<String> failures = new ArrayList<>();

        for (final ControllerDecision required : sc.getRequiredDecisions()) {
            boolean seen = false;
            for (final ControllerDecision obs : observed) {
                if (obs == required) { seen = true; break; }
            }
            if (!seen) {
                failures.add("Expected decision " + required.name()
                    + " was never observed. Observed: " + summarise(observed));
            }
        }

        for (final ControllerDecision forbidden : sc.getForbiddenDecisions()) {
            for (final ControllerDecision obs : observed) {
                if (obs == forbidden) {
                    failures.add("Forbidden decision " + forbidden.name()
                        + " was observed during the run.");
                    break;
                }
            }
        }

        if (sc.isExpectBrakingEngaged() && maxBrakeIntensity <= 0.0f) {
            failures.add("Autonomous braking was expected (intensity > 0) but BrakeActuator"
                + " stayed at 0.0 throughout the run.");
        }

        if (sc.isExpectNoBraking() && maxBrakeIntensity > 0.0f) {
            failures.add("No braking was expected but BrakeActuator reached intensity="
                + maxBrakeIntensity + " during the run.");
        }

        if (observed.isEmpty()) {
            failures.add("No decisions were observed at all — controller may not have started.");
        }

        final long wallEnd = System.currentTimeMillis();
        final boolean passed = failures.isEmpty();
        return new TestResult(sc, passed, failures, observed, maxBrakeIntensity,
                              wallEnd - wallStart);
    }

    // =========================================================================
    // Sensor construction helpers
    // =========================================================================

    private Camera buildCamera(final String id, final String dir,
                                final FaultType fault, final int delay) {
        if (fault == FaultType.NONE) {
            return new Camera(id, dir);
        }
        return new FaultInjector.FaultyCamera(id, dir, fault, delay);
    }

    private RadarLidarSensor wrapRadarLidar(final RadarLidarSensor real,
                                             final FaultType fault,
                                             final int delay,
                                             final String label) {
        if (fault == FaultType.NONE) {
            return real;
        }
        return new FaultInjector.FaultyRadarLidar(real, fault, delay, label);
    }

    private WheelSensor buildWheel(final String id, final String dir,
                                    final FaultType fault, final int delay) {
        if (fault == FaultType.NONE) {
            return new WheelSensor(id, dir);
        }
        return new FaultInjector.FaultyWheelSensor(id, dir, fault, delay);
    }

    private Driver buildDriver(final String dir, final FaultType fault) {
        if (fault == FaultType.NONE) {
            return new Driver(dir);
        }
        return new FaultInjector.FaultyDriver(dir, fault);
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private String resolveScenarioDir(final String scenarioId) {
        final File dir = new File(System.getProperty("user.dir"),
                                  SCENARIOS_ROOT + scenarioId);
        return dir.exists() ? dir.getAbsolutePath() : null;
    }

    private static void sleep(final int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    private static String summarise(final List<ControllerDecision> list) {
        if (list.isEmpty()) { return "(none)"; }
        final StringBuilder sb = new StringBuilder("[");
        final int MAX = 10;
        for (int i = 0; i < Math.min(list.size(), MAX); i++) {
            if (i > 0) { sb.append(", "); }
            sb.append(list.get(i).name());
        }
        if (list.size() > MAX) { sb.append(", … +").append(list.size() - MAX); }
        sb.append("]");
        return sb.toString();
    }

    private static TestResult fail(final TestScenario sc, final long wallStart,
                                    final String reason) {
        final List<String> failures = new ArrayList<>();
        failures.add(reason);
        return new TestResult(sc, false, failures,
                              new ArrayList<>(), 0.0f,
                              System.currentTimeMillis() - wallStart);
    }
}
