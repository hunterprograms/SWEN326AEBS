package swen326.group4.Sensors.Camera;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Camera sensor implementation for the AEBS.
 *
 * Mirrors the Lidar sensor pattern. On start(), openAndReadMetadata() opens
 * the scenario JSON file and reads the weatherFactor from the metadata block.
 * weatherFactor is applied as a confidence modifier — camera classification
 * degrades under adverse weather conditions (NF2202).
 *
 * The file is held open after metadata is consumed. On each 50ms clock tick,
 * captureFrame() streams only the next tick entry from the file and produces
 * a CameraReading stamped with the current confidence.
 *
 * No more than one tick is held in memory at any time (Power of Ten rule 2).
 *
 * Confidence derivation for Camera:
 *   confidenceScore = weatherFactor
 *   where 0.0 = sensor blind, 1.0 = full visibility.
 *
 * Requirement traceability:
 *   NF2201 : Three Camera instances feed the CameraVoter 2oo3 architecture.
 *   NF2202 : OBSTRUCTED/DEGRADED status triggered by low weatherFactor.
 *   NF4201 : Malformed tick data sets status FAILED, excludes from vote.
 */
public class Camera {

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    /**
     * Classification of detected road objects.
     * Matches objectType values in worldCameraData.json scenario files.
     */
    public enum ObjectType {
        NONE,
        VEHICLE,
        PEDESTRIAN,
        CYCLIST,
        STATIONARY_OBJECT,
        UNKNOWN
    }

    /**
     * Operational state of this camera unit.
     * Consumed by CameraVoter to determine voting eligibility.
     */
    public enum SensorStatus {
        OK,           // normal function - votes in 2oo3
        DEGRADED,     // reduced confidence - still votes in 2oo3
        OBSTRUCTED,   // NF2202: excluded from 2oo3 vote, fallback active
        FAILED        // terminal - excluded from vote, driver alerted
    }

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /** Update frequency per spec section 4.2: every 50ms */
    private static final int CLOCK_INTERVAL_MS = 50;

    /** Below this confidence the camera is OBSTRUCTED per NF2202 */
    private static final float OBSTRUCTED_THRESHOLD = 0.2f;

    /** Below this confidence the camera is DEGRADED */
    private static final float DEGRADED_THRESHOLD = 0.5f;

    /** Suffix appended to sensorId to form the scenario filename */
    private static final String DATA_FILE = "worldCameraData.json";

    /** Maximum detected objects per frame - bounded per Power of Ten rule 2 */
    public static final int MAX_OBJECTS = 10;

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Sensor ID: "1", "2", or "3" — matches Camera(1,2,3) in architecture */
    private final String sensorId;

    /** Full path to this sensor's scenario data file */
    private final String dataFilePath;

    /** Scenario-wide confidence derived from weatherFactor in metadata */
    private float confidenceScore;

    /** Open reader held across all tick reads — closed on stop() */
    private BufferedReader reader;

    /** True once all ticks in the scenario file have been consumed */
    private boolean ticksExhausted;

    /** 50ms timer driving captureFrame() calls */
    private Timer clock;

    /** Most recent reading produced by captureFrame() */
    private CameraReading latestReading;

    /** Current operational status of this camera unit */
    private SensorStatus status;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Constructs a Camera sensor unit.
     * @param sensorId      "1", "2", or "3"
     * @param dataDirectory path to the directory containing scenario JSON files
     */
    public Camera(final String sensorId, final String dataDirectory) {
        assert sensorId != null : "sensorId must not be null";
        assert dataDirectory != null : "dataDirectory must not be null";
        this.sensorId       = sensorId;
        this.dataFilePath   = dataDirectory + "/" + sensorId + DATA_FILE;
        this.status         = SensorStatus.OK;
        this.latestReading  = null;
        this.ticksExhausted = false;
        this.confidenceScore = 1.0f;
    }

    // -------------------------------------------------------------------------
    // Public interface - mirrors Lidar
    // -------------------------------------------------------------------------

    /** Opens the scenario file, reads metadata, and starts the 50ms clock. */
    public void start() {
        openAndReadMetadata();
        if (status != SensorStatus.FAILED) {
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

    /** Returns the most recent CameraReading, or null if not yet available. */
    public CameraReading getLatestReading() {
        return latestReading;
    }

    /** Returns the current operational status of this camera unit. */
    public SensorStatus getStatus() {
        return status;
    }

    /**
     * Returns true if this camera should be included in the CameraVoter 2oo3 vote.
     * OBSTRUCTED and FAILED cameras are excluded per NF2201/NF2202.
     */
    public boolean isVotingEligible() {
        assert status != null : "status must not be null";
        assert sensorId != null : "sensorId must not be null";
        return status == SensorStatus.OK || status == SensorStatus.DEGRADED;
    }

    // -------------------------------------------------------------------------
    // Core tick method - called every 50ms by clock
    // -------------------------------------------------------------------------

    /**
     * Streams the next tick from the open scenario file and produces a
     * CameraReading. Sets status FAILED if ticks are exhausted or the
     * tick entry cannot be parsed (NF4201).
     */
    private void captureFrame() {
        assert !ticksExhausted || status == SensorStatus.FAILED : "exhausted ticks must set FAILED";

        if (ticksExhausted) {
            status = SensorStatus.FAILED;
            latestReading = null;
            return;
        }

        final String raw = readNextTickObject();

        if (raw == null) {
            ticksExhausted = true;
            status = SensorStatus.FAILED;
            latestReading = null;
            return;
        }

        try {
            final JSONObject tick = new JSONObject(raw);
            final long timestamp = tick.optLong("timestampMs", System.currentTimeMillis());
            final JSONArray objects = tick.getJSONArray("detectedObjects");

            final ObjectType[] detectedObjects = new ObjectType[MAX_OBJECTS];
            final float[] confidenceScores = new float[MAX_OBJECTS];
            int count = 0;

            // Initialise arrays to safe defaults - Power of Ten rule 2
            for (int i = 0; i < MAX_OBJECTS; i++) {
                detectedObjects[i] = ObjectType.NONE;
                confidenceScores[i] = 0.0f;
            }

            for (int i = 0; i < objects.length() && i < MAX_OBJECTS; i++) {
                final JSONObject obj = objects.getJSONObject(i);
                detectedObjects[i] = parseObjectType(obj.optString("objectType", "UNKNOWN"));
                confidenceScores[i] = confidenceScore;
                count++;
            }

            latestReading = new CameraReading(
                detectedObjects,
                confidenceScores,
                count,
                timestamp
            );

            System.out.println(
                "CameraSensor [" + sensorId + "]"
                + " t=" + timestamp
                + " count=" + count
                + " objects=" + buildObjectSummary(detectedObjects, count)
                + " conf=" + confidenceScore
                + " status=" + status
            );

        } catch (Exception e) {
            System.err.println("CameraSensor [" + sensorId + "]: malformed tick — "
                + e.getMessage());
            status = SensorStatus.FAILED;
            latestReading = null;
        }
    }

    // -------------------------------------------------------------------------
    // Metadata and file reading - mirrors Lidar pattern
    // -------------------------------------------------------------------------

    /**
     * Opens the scenario file and reads only the metadata block.
     * Extracts weatherFactor and derives confidenceScore and initial status.
     * Advances the reader past the ticks array opening for captureFrame().
     *
     * NF2202: low weatherFactor triggers OBSTRUCTED state.
     */
    private void openAndReadMetadata() {
        try {
            reader = new BufferedReader(new FileReader(dataFilePath));
            final String metadataRaw = streamNextObject();

            if (metadataRaw == null) {
                System.err.println("CameraSensor [" + sensorId + "]: missing metadata block.");
                status = SensorStatus.FAILED;
                return;
            }

            final JSONObject metadata = new JSONObject(metadataRaw);
            final float weatherFactor = (float) metadata.optDouble("weatherFactor", 1.0);
            this.confidenceScore = weatherFactor;
            updateStatus(confidenceScore);
            advancePastTicksOpening();

        } catch (IOException e) {
            System.err.println("CameraSensor [" + sensorId + "]: failed to open "
                + dataFilePath + " — " + e.getMessage());
            status = SensorStatus.FAILED;
        }
    }

    /**
     * Streams lines from the open reader until a complete JSON object is
     * accumulated. Mirrors Lidar.streamNextObject() exactly.
     */
    private String streamNextObject() throws IOException {
        assert reader != null : "reader must not be null when streaming";

        final StringBuilder builder = new StringBuilder();
        String line;
        int braceDepth = 0;
        boolean started = false;
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
                }
                if (braceDepth == 0) {
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
     * Advances the reader past the "ticks" key and its opening bracket
     * so captureFrame() lands on tick index zero.
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
     * Wraps streamNextObject() to suppress checked IOException for castRays().
     */
    private String readNextTickObject() {
        try {
            return streamNextObject();
        } catch (IOException e) {
            System.err.println("CameraSensor [" + sensorId + "]: error reading tick — "
                + e.getMessage());
            return null;
        }
    }

    // -------------------------------------------------------------------------
    // State management
    // -------------------------------------------------------------------------

    /**
     * Maps confidenceScore to SensorStatus.
     * NF2202: low weather confidence triggers OBSTRUCTED, excluding from vote.
     * NF4201: FAILED excludes this sensor from the 2oo3 vote entirely.
     */
    private void updateStatus(final float confidence) {
        assert !Float.isNaN(confidence) : "confidence must not be NaN";
        assert !Float.isInfinite(confidence) : "confidence must not be infinite";
        if (confidence <= OBSTRUCTED_THRESHOLD) {
            status = SensorStatus.OBSTRUCTED;
        } else if (confidence <= DEGRADED_THRESHOLD) {
            status = SensorStatus.DEGRADED;
        } else {
            status = SensorStatus.OK;
        }
        assert status != null : "status must not be null after updateStatus()";
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Parses a string objectType from JSON into the ObjectType enum.
     * Defaults to UNKNOWN for unrecognised values — never throws (NF4201).
     */
    private ObjectType parseObjectType(final String raw) {
        assert raw != null : "raw objectType string must not be null";
        if (raw.equalsIgnoreCase("VEHICLE"))          { return ObjectType.VEHICLE; }
        if (raw.equalsIgnoreCase("PEDESTRIAN"))       { return ObjectType.PEDESTRIAN; }
        if (raw.equalsIgnoreCase("CYCLIST"))          { return ObjectType.CYCLIST; }
        if (raw.equalsIgnoreCase("STATIONARY_OBJECT")){ return ObjectType.STATIONARY_OBJECT; }
        if (raw.equalsIgnoreCase("NONE"))             { return ObjectType.NONE; }
        return ObjectType.UNKNOWN;
    }

    private void startClock() {
        assert status != SensorStatus.FAILED : "must not start clock when FAILED";
        clock = new Timer("CameraSensor-" + sensorId, false);
        clock.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                captureFrame();
            }
        }, 0, CLOCK_INTERVAL_MS);
        System.out.println("CameraSensor [" + sensorId + "] timer started.");
    }

    private void closeReader() {
        if (reader != null) {
            try {
                reader.close();
            } catch (IOException e) {
                System.err.println("CameraSensor [" + sensorId + "]: error closing reader — "
                    + e.getMessage());
            }
        }
    }

    private String buildObjectSummary(final Camera.ObjectType[] objects, final int count) {
    assert objects != null : "objects must not be null";
    assert count >= 0 : "count must not be negative";
    final StringBuilder sb = new StringBuilder("[");
    for (int i = 0; i < count; i++) {
        sb.append(objects[i]);
        if (i < count - 1) { sb.append(", "); }
    }
    sb.append("]");
    return sb.toString();
}
}