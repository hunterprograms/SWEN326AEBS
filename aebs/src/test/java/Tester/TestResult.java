package Tester;

import swen326.group4.BrakingController.ControllerDecision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * TestResult — captures the outcome of a single TestScenario execution.
 *
 * Populated by AEBSTestRunner after each scenario completes. Contains:
 *   - the original TestScenario definition
 *   - whether the test passed or failed
 *   - a list of failure reasons (empty on pass)
 *   - the full ordered list of ControllerDecisions observed during the run
 *   - the maximum BrakeActuator intensity reached
 *   - wall-clock duration of the test execution
 */
public final class TestResult {

    private final TestScenario scenario;
    private final boolean      passed;
    private final List<String> failureReasons;
    private final List<ControllerDecision> observedDecisions;
    private final float        maxBrakeIntensity;
    private final long         wallClockMs;

    /**
     * @param scenario          the scenario that was run
     * @param passed            true if all assertions held
     * @param failureReasons    human-readable explanations for each failure
     * @param observedDecisions every decision sampled during the run (in order)
     * @param maxBrakeIntensity highest BrakeActuator intensity seen
     * @param wallClockMs       real elapsed time in ms
     */
    public TestResult(final TestScenario scenario,
                      final boolean passed,
                      final List<String> failureReasons,
                      final List<ControllerDecision> observedDecisions,
                      final float maxBrakeIntensity,
                      final long wallClockMs) {
        assert scenario          != null : "scenario must not be null";
        assert failureReasons    != null : "failureReasons must not be null";
        assert observedDecisions != null : "observedDecisions must not be null";

        this.scenario          = scenario;
        this.passed            = passed;
        this.failureReasons    = Collections.unmodifiableList(new ArrayList<>(failureReasons));
        this.observedDecisions = Collections.unmodifiableList(new ArrayList<>(observedDecisions));
        this.maxBrakeIntensity = maxBrakeIntensity;
        this.wallClockMs       = wallClockMs;
    }

    public TestScenario getScenario()                     { return scenario; }
    public boolean isPassed()                             { return passed; }
    public List<String> getFailureReasons()               { return failureReasons; }
    public List<ControllerDecision> getObservedDecisions(){ return observedDecisions; }
    public float getMaxBrakeIntensity()                   { return maxBrakeIntensity; }
    public long getWallClockMs()                          { return wallClockMs; }

    /** Convenience: how many times a decision was observed. */
    public int countDecision(final ControllerDecision d) {
        int n = 0;
        for (final ControllerDecision obs : observedDecisions) {
            if (obs == d) { n++; }
        }
        return n;
    }

    /** True if the given decision appeared at least once. */
    public boolean sawDecision(final ControllerDecision d) {
        return countDecision(d) > 0;
    }

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder();
        sb.append(passed ? "PASS" : "FAIL")
          .append(" | ").append(scenario.getTestId())
          .append(" | ").append(scenario.getDescription())
          .append(" | maxBrake=").append(String.format("%.2f", maxBrakeIntensity))
          .append(" | ").append(wallClockMs).append("ms");
        if (!passed) {
            for (final String reason : failureReasons) {
                sb.append("\n       ✗ ").append(reason);
            }
        }
        return sb.toString();
    }

    // =========================================================================
    // TestReport — aggregate of all test results for a suite run
    // =========================================================================

    /**
     * TestReport — produced by AEBSTestRunner after running all scenarios.
     *
     * Provides a summary (pass/fail counts, coverage map by decision type)
     * and a formatted printable report.
     */
    public static final class TestReport {

        private final List<TestResult> results;
        private final long             totalWallClockMs;

        public TestReport(final List<TestResult> results, final long totalWallClockMs) {
            assert results != null : "results must not be null";
            this.results          = Collections.unmodifiableList(new ArrayList<>(results));
            this.totalWallClockMs = totalWallClockMs;
        }

        public List<TestResult> getResults()     { return results; }
        public long getTotalWallClockMs()         { return totalWallClockMs; }

        public int passCount() {
            int n = 0;
            for (final TestResult r : results) { if (r.isPassed()) n++; }
            return n;
        }

        public int failCount() {
            return results.size() - passCount();
        }

        /**
         * Returns a map of how many times each ControllerDecision was observed
         * across all test runs combined.
         */
        public Map<ControllerDecision, Integer> decisionCoverage() {
            final Map<ControllerDecision, Integer> map =
                new EnumMap<>(ControllerDecision.class);
            for (final ControllerDecision d : ControllerDecision.values()) {
                map.put(d, 0);
            }
            for (final TestResult r : results) {
                for (final ControllerDecision obs : r.getObservedDecisions()) {
                    map.put(obs, map.get(obs) + 1);
                }
            }
            return map;
        }

        /**
         * Returns a formatted multi-line report string suitable for writing to
         * a file or printing to the console.
         */
        public String toFormattedReport() {
            final StringBuilder sb = new StringBuilder();
            sb.append("╔══════════════════════════════════════════════════════════╗\n");
            sb.append("║             AEBS Test Suite — Results Report             ║\n");
            sb.append("╚══════════════════════════════════════════════════════════╝\n");
            sb.append(String.format("  Total tests : %d%n", results.size()));
            sb.append(String.format("  Passed      : %d%n", passCount()));
            sb.append(String.format("  Failed      : %d%n", failCount()));
            sb.append(String.format("  Wall clock  : %d ms%n%n", totalWallClockMs));

            sb.append("──────────────────────────────────────────────────────────\n");
            sb.append("  Decision Coverage (total observations across all tests)\n");
            sb.append("──────────────────────────────────────────────────────────\n");
            for (final Map.Entry<ControllerDecision, Integer> entry
                    : decisionCoverage().entrySet()) {
                sb.append(String.format("  %-22s : %d%n",
                    entry.getKey().name(), entry.getValue()));
            }
            sb.append("\n");

            sb.append("──────────────────────────────────────────────────────────\n");
            sb.append("  Individual Test Results\n");
            sb.append("──────────────────────────────────────────────────────────\n");
            for (final TestResult r : results) {
                sb.append(r.toString()).append("\n");
                if (r.isPassed()) {
                    /* Show the decision sequence on pass for auditability. */
                    sb.append("       Decisions: ");
                    final List<ControllerDecision> decs = r.getObservedDecisions();
                    final int MAX_SHOW = 20;
                    for (int i = 0; i < Math.min(decs.size(), MAX_SHOW); i++) {
                        sb.append(decs.get(i).name());
                        if (i < decs.size() - 1 && i < MAX_SHOW - 1) {
                            sb.append(" → ");
                        }
                    }
                    if (decs.size() > MAX_SHOW) {
                        sb.append(" … (" + decs.size() + " total)");
                    }
                    sb.append("\n");
                }
                sb.append("\n");
            }

            sb.append("══════════════════════════════════════════════════════════\n");
            sb.append(failCount() == 0
                ? "  ✔  ALL TESTS PASSED\n"
                : "  ✘  " + failCount() + " TEST(S) FAILED — see details above\n");
            sb.append("══════════════════════════════════════════════════════════\n");
            return sb.toString();
        }
    }
}
