package swen326.group4.Sensors.Wheel_Sensor;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.stream.Collectors;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * WheelSensor
 *
 * Reads wheel speed data from worldWheelSpeedData.json every 10ms.
 * Implements:
 *   - FR2208 : 2oo3 voting across three redundant sensor sets (called externally via vote())
 *   - FR2209 : Signal continuity monitoring — flags sensors silent for 2+ consecutive intervals
 *   - FR2210 : Post-dropout plausibility check — holds restored sensor in stabilisation period
 *   - FR4203 : Frozen value detection via rate-of-change monitoring
 *   - NFR2203 : Persistent dropout log (timestamp, duration, deviation magnitude)
 *
 * Data format (worldWheelSpeedData.json) — one frame object per line inside
 * the "frames" array. The file is opened once and read one line at a time on
 * each readRPM() call, emulating live sensor input from the simulator.
 *
 * Wheel index order: 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
 */
public class WheelSensor {

    /* ------------------------------------------------------------------ */
    /* Constants                                                            */
    /* ------------------------------------------------------------------ */

    private static final int WHEEL_COUNT = 4;

    /** Wheel name labels used in console output. */
    private static final String[] WHEEL_NAMES = { "FL", "FR", "RL", "RR" };

    private static void printSummary(final int ticks, final WheelSensor[] sensors) {
        System.out.println("-".repeat(62));
        System.out.println("Simulation complete. " + ticks + " ticks processed.");
        for (final WheelSensor s : sensors) {
            System.out.println("\n--- Sensor " + s.getSensorId() + " Summary ---");
            System.out.println("  Accepted readings : " + s.log.size());
            for (int w = 0; w < WHEEL_COUNT; w++) {
                final String state;
                if (s.flagged[w])          { state = "FLAGGED"; }
                else if (s.frozen[w])      { state = "FROZEN"; }
                else if (s.inStabilisation[w]) { state = "STABILISING"; }
                else                       { state = "OK"; }
                System.out.printf("  %s final state     : %s%n", WHEEL_NAMES[w], state);
            }
            final HashMap<Integer, float[]> dropouts = s.getDropoutLog();
            if (dropouts.isEmpty()) {
                System.out.println("  Dropout events    : none");
            } else {
                System.out.println("  Dropout events    : " + dropouts.size());
                for (Map.Entry<Integer, float[]> e : dropouts.entrySet()) {
                    System.out.printf("    t=%dms  duration=%dms  deviation=%.1f RPM%n",
                            (int) e.getKey(), (int) e.getValue()[0], e.getValue()[1]);
                }
            }
            s.close();
        }
    }

    private static String formatRpm(final float rpm) {
        return rpm < 0 ? "N/A" : String.format("%.1f", rpm);
    }

    /** Update interval specified by the system (ms). */
    private static final int UPDATE_INTERVAL_MS = 10;

    /**
     * Number of consecutive silent intervals before a sensor is flagged
     * as unavailable (FR2209).
     */
    private static final int DROPOUT_THRESHOLD_INTERVALS = 2;

    /**
     * RPM deviation threshold above which a restored sensor reading is
     * considered implausible and the sensor is held in stabilisation (FR2210).
     */
    private static final float PLAUSIBILITY_THRESHOLD_RPM = 50.0f;

    /**
     * Number of consecutive valid intervals a restored sensor must pass
     * before being re-integrated into braking decisions (FR2210).
     */
    private static final int STABILISATION_INTERVALS = 5;

    /**
     * If RPM change across consecutive intervals is less than this value
     * while vehicle speed is non-trivial, the sensor is considered frozen (FR4203).
     */
    private static final float FROZEN_DELTA_THRESHOLD = 0.01f;

    /**
     * Number of consecutive near-zero-delta intervals before a sensor is
     * classified as frozen (FR4203).
     */
    private static final int FROZEN_COUNT_THRESHOLD = 3;

    /** Minimum RPM above which frozen-value detection is active. */
    private static final float FROZEN_MIN_ACTIVE_RPM = 10.0f;

    /* ------------------------------------------------------------------ */
    /* State                                                                */
    /* ------------------------------------------------------------------ */

    /** Which sensor set this instance represents (1, 2, or 3 for 2oo3). */
    private final int sensorId;

    /** Path to the world data file — retained so start() can derive sibling paths. */
    private final String dataFile;

    /** Open stream into the simulator world file — read one frame per tick. */
    private final BufferedReader reader;

    /** Whether the stream has been exhausted or closed. */
    private boolean exhausted = false;

    /** Validated log of readings accepted into braking decisions. */
    private final List<WheelSensorData> log = new ArrayList<>();

    /* --- Dropout tracking (FR2209) --- */

    /** How many consecutive missed intervals each wheel has accumulated. */
    private final int[] dropoutCount = new int[WHEEL_COUNT];

    /** Whether each wheel is currently flagged as unavailable. */
    private final boolean[] flagged = new boolean[WHEEL_COUNT];

    /* --- Stabilisation tracking (FR2210) --- */

    /** Consecutive valid intervals since a wheel was restored. */
    private final int[] stabilisationCount = new int[WHEEL_COUNT];

    /** Whether each wheel is in a post-dropout stabilisation period. */
    private final boolean[] inStabilisation = new boolean[WHEEL_COUNT];

    /** Last accepted RPM for each wheel (used for plausibility checks). */
    private final float[] lastAcceptedRpm = new float[WHEEL_COUNT];

    /* --- Frozen value detection (FR4203) --- */

    /** How many consecutive near-zero-delta intervals each wheel has seen. */
    private final int[] frozenCount = new int[WHEEL_COUNT];

    /** Whether each wheel is currently classified as frozen. */
    private final boolean[] frozen = new boolean[WHEEL_COUNT];

    /** RPM from the previous interval, for delta calculation. */
    private final float[] previousRpm = new float[WHEEL_COUNT];

    /* --- Dropout log (NFR2203) --- */

    /** Persistent log entry: [timestamp_ms, duration_ms, deviation_rpm]. */
    private final HashMap<Integer, float[]> dropoutLog = new HashMap<>();

    /** Timestamp at which each wheel dropout began. */
    private final int[] dropoutStartMs = new int[WHEEL_COUNT];

    /** Current simulated timestamp, incremented each readRPM() call. */
    private int currentTimestampMs = 0;

    /* --- Timer-driven mode --- */

    /** Voter used when this sensor is the entry point (start() mode). */
    private WheelSensorVoter voter = null;

    /** Timer that drives the 10ms polling loop in start() mode. */
    private Timer timer = null;

    /* ------------------------------------------------------------------ */
    /* Constructor                                                          */
    /* ------------------------------------------------------------------ */

    /**
     * Constructs a WheelSensor for the given sensor set ID and opens the
     * simulator world file for streaming. The file is read one frame at a
     * time on each readRPM() call rather than loaded into memory up front.
     *
     * @param sensorId  1, 2, or 3 — identifies this set within the 2oo3 trio
     * @param dataFile  path to the simulator JSON world file
     * @throws IOException if the file cannot be opened
     */
    public WheelSensor(final int sensorId, final String dataFile) throws IOException {
        this.sensorId = sensorId;
        this.dataFile = dataFile;
        this.reader = new BufferedReader(new FileReader(dataFile));
        skipToFrames();
    }

    /* ------------------------------------------------------------------ */
    /* Public API                                                           */
    /* ------------------------------------------------------------------ */

    /**
     * Reads the next frame from the world file, applies all validation
     * checks, and stores the result in the log if accepted.
     *
     * Called every 10ms by the controller.
     *
     * @return true if a valid, accepted reading was produced; false if the
     *         sensor has no more data or all wheels are flagged/frozen
     */
    public boolean readRPM() {
        if (exhausted) {
            return false;
        }

        final float[] raw = readNextFrame();
        if (raw == null) {
            exhausted = true;
            return false;
        }

        final float[] validated = new float[WHEEL_COUNT];
        boolean anyWheelUsable = false;

        for (int i = 0; i < WHEEL_COUNT; i++) {
            final float rpm = raw[i];

            /* --- FR2209: dropout detection --- */
            if (rpm < 0) {
                /* Negative RPM signals a missing/null reading from simulator. */
                dropoutCount[i]++;
                if (dropoutCount[i] >= DROPOUT_THRESHOLD_INTERVALS && !flagged[i]) {
                    flagged[i] = true;
                    dropoutStartMs[i] = currentTimestampMs;
                    /* Record the dropout event as soon as it is confirmed (NFR2203).
                     * Deviation is unknown until signal restores, so stored as -1
                     * and updated when the sensor comes back online. */
                    dropoutLog.put(dropoutStartMs[i], new float[]{-1, -1});
                }
                validated[i] = -1;
                continue;
            }

            /* Signal arrived — reset dropout counter. */
            if (flagged[i]) {
                /* --- FR2210: post-dropout plausibility check --- */
                final float deviation = Math.abs(rpm - lastAcceptedRpm[i]);
                
                /* Update the existing log entry with duration and deviation now
                 * that we know them — find the most recent entry for this wheel. */
                logDropout(i, deviation);

                if (deviation > PLAUSIBILITY_THRESHOLD_RPM) {
                    /* Implausible: hold in stabilisation. */
                    inStabilisation[i] = true;
                    stabilisationCount[i] = 0;
                } else {
                    flagged[i] = false;
                }
                dropoutCount[i] = 0;
            }

            if (inStabilisation[i]) {
                stabilisationCount[i]++;
                if (stabilisationCount[i] >= STABILISATION_INTERVALS) {
                    inStabilisation[i] = false;
                    flagged[i] = false;
                }
                validated[i] = -1;
                continue;
            }

            /* --- FR4203: frozen value detection --- */
            final float delta = Math.abs(rpm - previousRpm[i]);
            if (rpm > FROZEN_MIN_ACTIVE_RPM && delta < FROZEN_DELTA_THRESHOLD) {
                frozenCount[i]++;
                if (frozenCount[i] >= FROZEN_COUNT_THRESHOLD) {
                    frozen[i] = true;
                    validated[i] = -1;
                    continue;
                }
            } else {
                frozenCount[i] = 0;
                frozen[i] = false;
            }

            previousRpm[i] = rpm;
            lastAcceptedRpm[i] = rpm;
            validated[i] = rpm;
            anyWheelUsable = true;
        }

        if (anyWheelUsable) {
            log.add(new WheelSensorData(currentTimestampMs, validated, sensorId));
        }

        currentTimestampMs += UPDATE_INTERVAL_MS;
        return anyWheelUsable;
    }

    /** Returns the sensor set ID (1, 2, or 3). */
    public int getSensorId() {
        return sensorId;
    }

    /**
     * Starts the 10ms timer loop. This sensor acts as sensor 1; sensors 2
     * and 3 are opened automatically from the same directory and file
     * naming convention. The voter runs every tick and prints results until
     * data is exhausted, then prints a summary and stops.
     *
     * @throws IOException if sensor 2 or 3 world files cannot be opened
     */
    public void start() throws IOException {
        final String base = dataFile.replaceAll("\\d+\\.json$", "");

        final WheelSensor sensorB = new WheelSensor(2, base + "2.json");
        final WheelSensor sensorC = new WheelSensor(3, base + "3.json");
        final WheelSensor[] all   = { this, sensorB, sensorC };

        voter = new WheelSensorVoter(this, sensorB, sensorC);

        System.out.println("=== Wheel Speed Sensor Test ===");
        System.out.printf("%-8s  %-12s  %-12s  %-12s  %-12s%n",
                "Time(ms)", "FL (voted)", "FR (voted)", "RL (voted)", "RR (voted)");
        System.out.println("-".repeat(62));

        timer = new Timer("WheelSpeedSensor", true);

        timer.scheduleAtFixedRate(new TimerTask() {
            private int tick = 0;

            @Override
            public void run() {
                if (!voter.hasData()) {
                    timer.cancel();
                    printSummary(tick, all);
                    return;
                }

                final float[] voted = voter.vote();
                final int ts = tick * UPDATE_INTERVAL_MS;

                System.out.printf("%-8d  %-12s  %-12s  %-12s  %-12s%n",
                        ts,
                        formatRpm(voted[0]), formatRpm(voted[1]),
                        formatRpm(voted[2]), formatRpm(voted[3]));

                for (final WheelSensor s : all) {
                    for (int w = 0; w < WHEEL_COUNT; w++) {
                        if (s.isFlagged(w)) {
                            System.out.printf("  [S%d] %s FLAGGED (dropout)%n",
                                    s.getSensorId(), WHEEL_NAMES[w]);
                        }
                        if (s.isFrozen(w)) {
                            System.out.printf("  [S%d] %s FROZEN (no rate-of-change)%n",
                                    s.getSensorId(), WHEEL_NAMES[w]);
                        }
                        if (s.isInStabilisation(w)) {
                            System.out.printf("  [S%d] %s IN STABILISATION (post-dropout)%n",
                                    s.getSensorId(), WHEEL_NAMES[w]);
                        }
                    }
                }

                tick++;
            }
        }, 0, UPDATE_INTERVAL_MS);

        /* Block main thread until the timer finishes. */
        try {
            Thread.sleep(Long.MAX_VALUE);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Returns the most recent validated reading, or null if no reading has
     * been accepted yet.
     */
    public WheelSensorData getLatestReading() {
        if (log.isEmpty()) {
            return null;
        }
        return log.get(log.size() - 1);
    }

    /**
     * Returns all accepted readings for this sensor.
     */
    public List<WheelSensorData> getLog() {
        return new ArrayList<>(log);
    }

    /**
     * Returns the persistent dropout log (NFR2203).
     * Each entry is float[3]: [timestamp_ms, duration_ms, deviation_rpm].
     */
    public HashMap<Integer, float[]> getDropoutLog() {
        return new HashMap<>(dropoutLog);
    }

    /**
     * Returns whether wheel[i] is currently flagged as unavailable (FR2209).
     */
    public boolean isFlagged(final int wheelIndex) {
        return flagged[wheelIndex];
    }

    /**
     * Returns whether wheel[i] is currently classified as frozen (FR4203).
     */
    public boolean isFrozen(final int wheelIndex) {
        return frozen[wheelIndex];
    }

    /**
     * Returns whether wheel[i] is in a post-dropout stabilisation period (FR2210).
     */
    public boolean isInStabilisation(final int wheelIndex) {
        return inStabilisation[wheelIndex];
    }

    /**
     * Returns true if this sensor has not yet exhausted the world file.
     */
    public boolean hasData() {
        return !exhausted;
    }

    @Override
    public String toString() {
        return log.stream()
                .map(WheelSensorData::toString)
                .collect(Collectors.joining(", ", "[\n", "\n]"));
    }

    /* ------------------------------------------------------------------ */
    /* Private helpers                                                      */
    /* ------------------------------------------------------------------ */

    /**
     * Advances the reader past the metadata and opening "frames" array line
     * so that the next read lands on the first frame object.
     */
    private void skipToFrames() throws IOException {
        String line;
        while ((line = reader.readLine()) != null) {
            if (line.contains("\"frames\"")) {
                return;
            }
        }
        throw new IOException("No 'frames' array found in world data file.");
    }

    /**
     * Reads and parses the next frame line from the open world file.
     * Lines that are array punctuation ([ ] ,) or closing braces are skipped.
     *
     * @return float[4] of raw RPM values, or null if the stream is exhausted
     */
    private float[] readNextFrame() {
        try {
            String line;
            while ((line = reader.readLine()) != null) {
                final String trimmed = line.trim();

                /* Skip array brackets, closing brace, and bare commas. */
                if (trimmed.isEmpty()
                        || trimmed.equals("[")
                        || trimmed.equals("]")
                        || trimmed.equals("}")
                        || trimmed.equals(",")) {
                    continue;
                }

                /* Strip trailing comma so the line parses as valid JSON. */
                final String json = trimmed.endsWith(",")
                        ? trimmed.substring(0, trimmed.length() - 1)
                        : trimmed;

                /* Only process lines that look like frame objects. */
                if (!json.startsWith("{")) {
                    continue;
                }

                final JSONObject frame = new JSONObject(json);
                final JSONArray rpmArray = frame.getJSONArray("rpm");
                final float[] rpm = new float[WHEEL_COUNT];
                for (int w = 0; w < WHEEL_COUNT; w++) {
                    rpm[w] = (float) rpmArray.getDouble(w);
                }
                return rpm;
            }
        } catch (IOException e) {
            exhausted = true;
        }
        return null;
    }

    /**
     * Closes the underlying file reader. Should be called when the sensor
     * is no longer needed.
     */
    public void close() {
        try {
            reader.close();
        } catch (IOException e) {
            /* Nothing meaningful to do here. */
        }
    }

    /**
     * Records a dropout event in the persistent log (NFR2203).
     *
     * @param wheelIndex  which wheel experienced the dropout
     * @param deviation   RPM deviation observed on signal restoration
     */
    private void logDropout(final int wheelIndex, final float deviation) {
        final int duration = currentTimestampMs - dropoutStartMs[wheelIndex];
        dropoutLog.put(dropoutStartMs[wheelIndex], new float[]{duration, deviation});
    }
}