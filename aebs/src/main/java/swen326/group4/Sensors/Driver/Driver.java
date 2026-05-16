package swen326.group4.Sensors.Driver;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import org.json.JSONObject;

/**
 * Simulated driver input for the AEBS.
 *
 * Mirrors the Camera and Lidar sensor pattern. On start(),
 * openAndReadMetadata() opens the scenario JSON file and reads the metadata
 * block. On each 250ms clock tick, readNextAction() streams the next tick
 * entry and produces a DriverReading.
 *
 * The driver simulates a person driving — outputting BRAKE or NONE each tick
 * independently of sensor data. This allows testing scenarios where the driver
 * brakes correctly, brakes too late, or fails to brake entirely, which feeds
 * into HARA hazard scenarios for the Emergency Intervention Indicators.
 *
 * No more than one tick is held in memory at any time (Power of Ten rule 2).
 *
 * Requirement traceability:
 *   NF2201 : Driver input is one factor in AEBS collision decision
 *   HF-001 : Simulates driver startle / false brake response
 *   HF-002 : Simulates inattentive driver failing to brake
 */
public class Driver {

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    /**
     * Driver action for this tick.
     * BRAKE — driver is applying brakes.
     * NONE  — driver is not braking.
     */
    public enum Action {
        BRAKE,
        NONE
    }

    /**
     * Operational state of the driver simulator.
     */
    public enum DriverState {
        ACTIVE,   // reading ticks normally
        FINISHED, // all ticks exhausted
        FAILED    // file could not be opened or parsed
    }

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /** Driver reaction cycle — 250ms */
    private static final int CLOCK_INTERVAL_MS = 250;

    /** Scenario filename */
    private static final String DATA_FILE = "worldDriverData.json";

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Path to the driver scenario data file */
    private final String dataFilePath;

    /** Open reader held across all tick reads — closed on stop() */
    private BufferedReader reader;

    /** True once all ticks have been consumed */
    private boolean ticksExhausted;

    /** 250ms timer driving readNextAction() calls */
    private Timer clock;

    /** Most recent reading produced by readNextAction() */
    private DriverReading latestReading;

    /** Current state of the driver simulator */
    private DriverState state;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Constructs a Driver simulator.
     * @param dataDirectory path to the directory containing scenario JSON files
     */
    public Driver(final String dataDirectory) {
        assert dataDirectory != null : "dataDirectory must not be null";
        this.dataFilePath   = dataDirectory + "/" + DATA_FILE;
        this.state          = DriverState.ACTIVE;
        this.latestReading  = null;
        this.ticksExhausted = false;
    }

    // -------------------------------------------------------------------------
    // Public interface - mirrors Camera and Lidar
    // -------------------------------------------------------------------------

    /** Opens the scenario file, reads metadata, and starts the 250ms clock. */
    public void start() {
        openAndReadMetadata();
        if (state != DriverState.FAILED) {
            startClock();
        }
    }

    /** Stops the clock and closes the scenario file reader. */
    public void stop() {
        if (clock != null) {
            clock.cancel();
        }
        closeReader();
    }

    /** Returns the most recent DriverReading, or null if not yet available. */
    public DriverReading getLatestReading() {
        return latestReading;
    }

    /** Returns the current state of the driver simulator. */
    public DriverState getState() {
        return state;
    }

    // -------------------------------------------------------------------------
    // Core tick method - called every 250ms by clock
    // -------------------------------------------------------------------------

    /**
     * Streams the next tick from the open scenario file and produces a
     * DriverReading. Sets state FINISHED when ticks are exhausted.
     */
    private void readNextAction() {
        assert state != DriverState.FAILED : "must not tick when FAILED";

        if (ticksExhausted) {
            state = DriverState.FINISHED;
            latestReading = null;
            return;
        }

        final String raw = readNextTickObject();

        if (raw == null) {
            ticksExhausted = true;
            state = DriverState.FINISHED;
            latestReading = null;
            return;
        }

        try {
            final JSONObject tick = new JSONObject(raw);
            final long timestamp = tick.optLong("timestampMs", System.currentTimeMillis());
            final Action action  = parseAction(tick.optString("action", "NONE"));

            latestReading = new DriverReading(action, timestamp);

            System.out.println(
                "Driver t=" + timestamp
                + " action=" + action
                + " state=" + state
            );

        } catch (Exception e) {
            System.err.println("Driver: malformed tick — " + e.getMessage());
            state = DriverState.FAILED;
            latestReading = null;
        }
    }

    // -------------------------------------------------------------------------
    // Metadata and file reading - mirrors Camera pattern
    // -------------------------------------------------------------------------

    /**
     * Opens the scenario file and reads the metadata block.
     * Advances the reader past the ticks array opening for readNextAction().
     */
    private void openAndReadMetadata() {
        try {
            reader = new BufferedReader(new FileReader(dataFilePath));
            final String metadataRaw = streamNextObject();

            if (metadataRaw == null) {
                System.err.println("Driver: missing metadata block.");
                state = DriverState.FAILED;
                return;
            }

            final JSONObject outer = new JSONObject(metadataRaw);
            final JSONObject metadata = outer.optJSONObject("metadata");

            if (metadata == null) {
                System.err.println("Driver: missing metadata key.");
                state = DriverState.FAILED;
                return;
            }

            System.out.println("Driver scenario: " + metadata.optString("description", "unknown"));
            advancePastTicksOpening();

        } catch (IOException e) {
            System.err.println("Driver: failed to open " + dataFilePath
                + " — " + e.getMessage());
            state = DriverState.FAILED;
        }
    }

    /**
     * Streams lines from the open reader until a complete JSON object is
     * accumulated. Mirrors Camera.streamNextObject() exactly.
     */
    private String streamNextObject() throws IOException {
        assert reader != null : "reader must not be null when streaming";

        final StringBuilder builder = new StringBuilder();
        String line;
        int braceDepth  = 0;
        int squareDepth = 0;
        boolean started  = false;
        boolean metadata = false;

        while ((line = reader.readLine()) != null) {
            final String trimmed = line.trim();

            if (trimmed.contains("metadata")) {
                braceDepth--;
                metadata = true;
            }

            if (trimmed.equals("]}")) {
                return null;
            }

            if (trimmed.startsWith("{")) {
                started = true;
            }

            if (started) {
                builder.append(trimmed);
                for (char c : trimmed.toCharArray()) {
                    if (c == '{') { braceDepth++; }
                    if (c == '}') { braceDepth--; }
                    if (c == '[') { squareDepth++; }
                    if (c == ']') { squareDepth--; }
                }
                if (braceDepth == 0 && squareDepth == 0) {
                    String result = builder.toString();
                    if (result.endsWith(",")) {
                        result = result.substring(0, result.length() - 1);
                    }
                    if (metadata) {
                        result += '}';
                    }
                    assert result != null : "result must not be null";
                    assert !result.isEmpty() : "result must not be empty";
                    return result;
                }
            }
        }
        return null;
    }

    /**
     * Advances the reader past the ticks key and opening bracket
     * so readNextAction() lands on tick index zero.
     */
    private void advancePastTicksOpening() throws IOException {
        assert reader != null : "reader must not be null when advancing";
        String line;
        while ((line = reader.readLine()) != null) {
            final String trimmed = line.trim();
            if (trimmed.contains("\"ticks\"") || trimmed.equals("[")) {
                return;
            }
        }
    }

    /**
     * Wraps streamNextObject() to suppress checked IOException.
     */
    private String readNextTickObject() {
        try {
            return streamNextObject();
        } catch (IOException e) {
            System.err.println("Driver: error reading tick — " + e.getMessage());
            return null;
        }
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Parses action string from JSON into the Action enum.
     * Defaults to NONE for unrecognised values — never throws.
     */
    private Action parseAction(final String raw) {
        assert raw != null : "raw action string must not be null";
        if (raw.equalsIgnoreCase("BRAKE")) { return Action.BRAKE; }
        return Action.NONE;
    }

    private void startClock() {
        assert state != DriverState.FAILED : "must not start clock when FAILED";
        clock = new Timer("Driver", false);
        clock.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                readNextAction();
            }
        }, 0, CLOCK_INTERVAL_MS);
        System.out.println("Driver timer started.");
    }

    private void closeReader() {
        if (reader != null) {
            try {
                reader.close();
            } catch (IOException e) {
                System.err.println("Driver: error closing reader — " + e.getMessage());
            }
        }
    }
} 