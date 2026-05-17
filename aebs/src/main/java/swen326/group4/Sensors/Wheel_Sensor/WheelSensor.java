package swen326.group4.Sensors.Wheel_Sensor;
 
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.stream.Collectors;
 
import org.json.JSONArray;
import org.json.JSONObject;
 
/**
 * WheelSensor
 *
 * A single redundant wheel speed sensor instance. Three of these are
 * instantiated by the controller for 2oo3 voting.
 *
 * Each instance owns its own Timer. Calling start() opens the world file,
 * skips to the frames array, and schedules a 10ms tick that calls readRPM()
 * on each interval. Calling stop() cancels the timer and closes the reader.
 *
 * Implements:
 *   FR2209  : Signal continuity monitoring — flags wheels silent for 2+
 *             consecutive intervals as unavailable
 *   FR2210  : Post-dropout plausibility check — holds restored wheels in a
 *             stabilisation period before re-integrating
 *   FR4203  : Frozen value detection via rate-of-change monitoring
 *   NFR2203 : Persistent dropout log (timestamp, duration, deviation)
 *
 * Data format: one JSON frame object per line inside a "frames" array.
 * Wheel index order: 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
 */
public class WheelSensor {
 
    /* ------------------------------------------------------------------ */
    /* Constants                                                            */
    /* ------------------------------------------------------------------ */
 
    /** Number of wheels per sensor set. */
    private static final int WHEEL_COUNT = 4;
 
    /** Update interval specified by the system (ms). */
    private static final int UPDATE_INTERVAL_MS = 10;
 
    /** Wheel name labels used in console output. */
    private static final String[] WHEEL_NAMES = { "FL", "FR", "RL", "RR" };
 
    /**
     * Number of consecutive silent intervals before a wheel is flagged
     * as unavailable (FR2209).
     */
    private static final int DROPOUT_THRESHOLD_INTERVALS = 2;
 
    /**
     * RPM deviation threshold above which a restored wheel reading is
     * considered implausible and held in stabilisation (FR2210).
     */
    private static final float PLAUSIBILITY_THRESHOLD_RPM = 50.0f;
 
    /**
     * Number of consecutive valid intervals a restored wheel must pass
     * before being re-integrated into braking decisions (FR2210).
     */
    private static final int STABILISATION_INTERVALS = 5;
 
    /**
     * If RPM change across consecutive intervals is less than this value
     * while the vehicle is moving, the wheel is considered frozen (FR4203).
     */
    private static final float FROZEN_DELTA_THRESHOLD = 0.01f;
 
    /**
     * Number of consecutive near-zero-delta intervals before a wheel is
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
 
    /** Path to the simulator world file for this sensor. */
    private final String dataFile;
 
    /** Open stream into the world file — one frame consumed per tick. */
    private BufferedReader reader;
 
    /** Whether the stream has been exhausted. */
    private boolean exhausted = false;
 
    /** Validated log of readings accepted into braking decisions. */
    private final List<WheelSensorData> log = new ArrayList<>();
 
    /** Most recent validated reading produced by this sensor. */
    private WheelSensorData latestReading = null;
 
    /** Timer driving the 10ms polling loop. */
    private Timer clock;
 
    /* --- Dropout tracking (FR2209) --- */
 
    private final int[]     dropoutCount  = new int[WHEEL_COUNT];
    private final boolean[] flagged       = new boolean[WHEEL_COUNT];
 
    /* --- Stabilisation tracking (FR2210) --- */
 
    private final int[]     stabilisationCount = new int[WHEEL_COUNT];
    private final boolean[] inStabilisation    = new boolean[WHEEL_COUNT];
    private final float[]   lastAcceptedRpm    = new float[WHEEL_COUNT];
 
    /* --- Frozen value detection (FR4203) --- */
 
    private final int[]     frozenCount  = new int[WHEEL_COUNT];
    private final boolean[] frozen       = new boolean[WHEEL_COUNT];
    private final float[]   previousRpm  = new float[WHEEL_COUNT];
 
    /* --- Dropout log (NFR2203) --- */
 
    private final List<float[]> dropoutLog    = new ArrayList<>();
    private final int[]         dropoutStartMs = new int[WHEEL_COUNT];
 
    /** Current simulated timestamp, incremented each readRPM() call. */
    private int currentTimestampMs = 0;
 
    /* ------------------------------------------------------------------ */
    /* Constructor                                                          */
    /* ------------------------------------------------------------------ */
 
    /**
     * @param sensorId  1, 2, or 3 — identifies this instance within the 2oo3 trio
     * @param dataFile  path to this sensor's simulator world file
     */
    public WheelSensor(final String sensorId, final String dataFile) {
        this.sensorId = Integer.parseInt(sensorId);
        this.dataFile = dataFile + "/worldWheelSpeedId"+sensorId+".json";
    }
 
    /* ------------------------------------------------------------------ */
    /* Lifecycle                                                            */
    /* ------------------------------------------------------------------ */
 
    /**
     * Opens the world file, skips to the frames array, and starts the 10ms
     * timer. Each tick calls readRPM() and updates latestReading.
     */
    public void start() {
        try {
            reader = new BufferedReader(new FileReader(dataFile));
            skipToFrames();
        } catch (IOException e) {
            System.err.println("WheelSensor [" + sensorId + "]: failed to open "
                    + dataFile + " — " + e.getMessage());
            return;
        }
 
        clock = new Timer("WheelSensor-" + sensorId, false);
        clock.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                if (exhausted) {
                    System.out.println(currentTimestampMs);
                    clock.cancel();
                    closeReader();
                    return;
                }
                readRPM();
            }
        }, 0, UPDATE_INTERVAL_MS);
    }
 
    /**
     * Cancels the timer and closes the file reader.
     */
    public void stop() {
        if (clock != null) {
            clock.cancel();
        }
        closeReader();
    }
 
    /* ------------------------------------------------------------------ */
    /* Public API                                                           */
    /* ------------------------------------------------------------------ */
 
    /**
     * Reads the next frame, applies all validation checks, and updates
     * latestReading. Called every 10ms by the internal timer.
     */
    public void readRPM() {
        if (exhausted) {
            return;
        }
 
        final float[] raw = readNextFrame();
        if (raw == null) {
            exhausted = true;
            return;
        }
        final float[] validated = new float[WHEEL_COUNT];
        boolean anyWheelUsable = false;
 
        for (int i = 0; i < WHEEL_COUNT; i++) {
            final float rpm = raw[i];
        
            /* --- FR2209: dropout detection --- */
            if (rpm < 0) {
                dropoutCount[i]++;
                if (dropoutCount[i] >= DROPOUT_THRESHOLD_INTERVALS && !flagged[i]) {
                    flagged[i] = true;
                    dropoutStartMs[i] = currentTimestampMs;
                    dropoutLog.add(new float[]{ dropoutStartMs[i], -1, -1 });
                    System.out.printf("WheelSensor [%d] %s FLAGGED (dropout) at t=%dms%n",
                            sensorId, WHEEL_NAMES[i], currentTimestampMs);
                }
                validated[i] = -1;
                continue;
            }
        
            /* Signal restored — run plausibility check (FR2210). */
            if (flagged[i]) {
                final float deviation = Math.abs(rpm - lastAcceptedRpm[i]);
                final int   duration  = currentTimestampMs - dropoutStartMs[i];
                updateDropoutLogEntry(dropoutStartMs[i], duration, deviation);
                if (deviation > PLAUSIBILITY_THRESHOLD_RPM) {
                    inStabilisation[i] = true;
                    stabilisationCount[i] = 0;
                    System.out.printf("WheelSensor [%d] %s IN STABILISATION at t=%dms (deviation=%.1f RPM)%n",
                            sensorId, WHEEL_NAMES[i], currentTimestampMs, deviation);
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
                    System.out.printf("WheelSensor [%d] %s restored at t=%dms%n",
                            sensorId, WHEEL_NAMES[i], currentTimestampMs);
                }
                validated[i] = -1;
                continue;
            }
        
            /* --- FR4203: frozen value detection --- */
            final float delta = Math.abs(rpm - previousRpm[i]);
            
            // Update previous raw RPM tracker BEFORE checking thresholds 
            // so we don't accidentally compare against old history frames
            float oldPrevious = previousRpm[i];
            previousRpm[i] = rpm; 
        
            if (rpm > FROZEN_MIN_ACTIVE_RPM && delta < FROZEN_DELTA_THRESHOLD) {
                frozenCount[i]++;
                if (frozenCount[i] >= FROZEN_COUNT_THRESHOLD) {
                    if (!frozen[i]) {
                        System.out.printf("WheelSensor [%d] %s FROZEN at t=%dms%n",
                                sensorId, WHEEL_NAMES[i], currentTimestampMs);
                    }
                    frozen[i] = true;
                    
                    // Core Change: Use a unique fallback marker (e.g., -2.0) for frozen 
                    // instead of -1.0 so your voting controller knows it's stuck, not missing!
                    validated[i] = -2.0f; 
                    continue;
                }
            } else {
                if (frozen[i]) {
                    System.out.printf("WheelSensor [%d] %s unfrozen at t=%dms%n",
                            sensorId, WHEEL_NAMES[i], currentTimestampMs);
                }
                frozenCount[i] = 0;
                frozen[i] = false;
            }
        
            lastAcceptedRpm[i] = rpm;
            validated[i] = rpm;
            anyWheelUsable = true;
        }
 
        if (anyWheelUsable) {
            latestReading = new WheelSensorData(currentTimestampMs, validated, sensorId);
            log.add(latestReading);
        }
        System.out.printf("Sensor %d | Time: %4dms | FL: %6.1f | FR: %6.1f | RL: %6.1f | RR: %6.1f%n",
        sensorId, 
        currentTimestampMs, 
        validated[0], 
        validated[1], 
        validated[2], 
        validated[3]);
        currentTimestampMs += UPDATE_INTERVAL_MS;
    }
 
    /** Returns the most recent validated reading, or null if none yet. */
    public WheelSensorData getLatestReading() {
        return latestReading;
    }
 
    /** Returns a copy of all accepted readings for this sensor. */
    public List<WheelSensorData> getLog() {
        return new ArrayList<>(log);
    }
 
    /**
     * Returns the persistent dropout log (NFR2203).
     * Each entry is float[3]: [timestamp_ms, duration_ms, deviation_rpm].
     */
    public List<float[]> getDropoutLog() {
        return new ArrayList<>(dropoutLog);
    }
 
    /** Returns the sensor ID (1, 2, or 3). */
    public int getSensorId() {
        return sensorId;
    }
 
    /** Returns whether the stream has been fully consumed. */
    public boolean isExhausted() {
        return exhausted;
    }
 
    public boolean isFlagged(final int wheelIndex)        { return flagged[wheelIndex]; }
    public boolean isFrozen(final int wheelIndex)         { return frozen[wheelIndex]; }
    public boolean isInStabilisation(final int wheelIndex){ return inStabilisation[wheelIndex]; }
 
    @Override
    public String toString() {
        return log.stream()
                .map(WheelSensorData::toString)
                .collect(Collectors.joining(", ", "[\n", "\n]"));
    }
 
    /* ------------------------------------------------------------------ */
    /* Private helpers                                                      */
    /* ------------------------------------------------------------------ */
 
    private void skipToFrames() throws IOException {
        String line;
        while ((line = reader.readLine()) != null) {
            if (line.contains("\"frames\"")) {
                return;
            }
        }
        throw new IOException("No 'frames' array found in " + dataFile);
    }
 
    private float[] readNextFrame() {
        try {
            String line;
            while ((line = reader.readLine()) != null) {
                final String trimmed = line.trim();
                if (trimmed.isEmpty()
                        || trimmed.equals("[")
                        || trimmed.equals("]")
                        || trimmed.equals("}")
                        || trimmed.equals(",")) {
                    continue;
                }
                final String json = trimmed.endsWith(",")
                        ? trimmed.substring(0, trimmed.length() - 1)
                        : trimmed;
                if (!json.startsWith("{")) {
                    continue;
                }
                final JSONObject frame    = new JSONObject(json);
                final JSONArray  rpmArray = frame.getJSONArray("rpm");
                final float[]    rpm      = new float[WHEEL_COUNT];
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
     * Updates the most recent dropout log entry for the given start timestamp
     * with the actual duration and deviation once the signal is restored.
     */
    private void updateDropoutLogEntry(final int startMs,
                                       final int duration,
                                       final float deviation) {
        for (int i = dropoutLog.size() - 1; i >= 0; i--) {
            if ((int) dropoutLog.get(i)[0] == startMs) {
                dropoutLog.get(i)[1] = duration;
                dropoutLog.get(i)[2] = deviation;
                return;
            }
        }
    }
 
    private void closeReader() {
        if (reader != null) {
            try {
                reader.close();
            } catch (IOException e) {
                /* Nothing meaningful to do. */
            }
        }
    }
}