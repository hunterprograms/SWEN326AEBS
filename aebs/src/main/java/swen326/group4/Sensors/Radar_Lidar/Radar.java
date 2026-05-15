package swen326.group4.Sensors.Radar_Lidar;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Radar sensor implementation.
 *
 * On start(), openAndReadMetadata() opens worldRadarData.json and streams
 * only the metadata block to extract the scenario-wide weatherFactor and
 * rfInterferenceFactor. These are combined into a single confidenceScore
 * that is stamped on every reading this sensor produces.
 *
 * The file is held open after metadata is consumed. On each 100ms clock
 * tick, readFrequencies() streams only the next tick entry from the file
 * and produces a RadarLidarReading stamped with the scenario-wide
 * confidenceScore. No more than one tick is held in memory at any time.
 *
 * Confidence derivation for radar:
 *   adjustedWeather = weatherFactor + (1 - weatherFactor) * WEATHER_RESISTANCE
 *   confidenceScore = min(rfInterferenceFactor, adjustedWeather)
 *
 * WEATHER_RESISTANCE means radar degrades less aggressively than LiDAR
 * in adverse weather. RF interference has no resistance modifier and is
 * the primary degradation source specific to radar.
 *
 * where 0.0 = sensor blind/jammed, 1.0 = full capability.
 *
 * Requirement traceability:
 *   FR2103 : radar resists weather better than LiDAR; DEGRADED shifts controller weight.
 *   FR2104 : FAILED status excludes this sensor from the 2oo3 vote entirely.
 *   FR2107 : low rfInterferenceFactor lowers confidence; controller must not act on radar alone.
 *   FR2108 : frequency hopping resistance simulated by rfInterferenceFactor being less
 *            severe than a raw jamming scenario would produce.
 */
public class Radar implements RadarLidarSensor {

    private static final int    CLOCK_INTERVAL_MS  = 100;
    private static final float  DEGRADED_THRESHOLD = 0.5f;
    private static final float  FAILED_THRESHOLD   = 0.1f;

    /**
     * FR2103: scales down the weather penalty applied to radar relative to LiDAR.
     * e.g. weatherFactor=0.4 -> adjustedWeather = 0.4 + (0.6 * 0.4) = 0.64
     */
    private static final float  WEATHER_RESISTANCE = 0.4f;
    private static final String DATA_FILE          = "worldRadarData.json";

    private final String      sensorId;
    private final String      dataFilePath;

    private float             confidenceScore;
    private BufferedReader    reader;
    private boolean           ticksExhausted;

    private Timer             clock;
    private RadarLidarReading latestReading;
    private SensorStatus      status;

    public Radar(String sensorId, String dataDirectory) {
        this.sensorId        = sensorId;
        this.dataFilePath    = dataDirectory + "/" + DATA_FILE;
        this.status          = SensorStatus.OK;
        this.latestReading   = null;
        this.ticksExhausted  = false;
        this.confidenceScore = 1.0f;
    }

    @Override
    public void start() {
        openAndReadMetadata();
        if (status != SensorStatus.FAILED) {
            startClock();
        }
    }

    @Override
    public void stop() {
        if (clock != null) {
            clock.cancel();
        }
        closeReader();
    }

    @Override
    public RadarLidarReading getLatestReading() {
        return latestReading;
    }

    @Override
    public SensorStatus getStatus() {
        return status;
    }

    /**
     * Called every 100ms by the internal clock.
     *
     * Streams the next tick object from the open file. Parses detectedObjects
     * and produces a RadarLidarReading stamped with the confidenceScore
     * established at initialisation. Status is not re-evaluated per tick —
     * weatherFactor and rfInterferenceFactor are scenario-wide constants
     * read once from metadata.
     *
     * Sets status FAILED and nulls latestReading if ticks are exhausted
     * or the tick entry cannot be parsed (FR2104).
     */
    private void readFrequencies() {
        if (ticksExhausted) {
            status = SensorStatus.FAILED;
            latestReading = null;
            return;
        }

        String raw = readNextTickObject();
        if (raw == null) {
            ticksExhausted = true;
            status = SensorStatus.FAILED;
            latestReading = null;
            return;
        }

        try {
            JSONObject tick      = new JSONObject(raw);
            long       timestamp = tick.optLong("timestampMs", System.currentTimeMillis());
            JSONArray  objects   = tick.getJSONArray("detectedObjects");

            List<DetectedObject> detected = new ArrayList<>();
            for (int i = 0; i < objects.length(); i++) {
                JSONObject obj = objects.getJSONObject(i);
                detected.add(new DetectedObject(
                    (float) obj.getDouble("distanceMetres"),
                    (float) obj.getDouble("relativeSpeedKmh"),
                    (float) obj.getDouble("bearingDegrees")
                ));
            }

            latestReading = new RadarLidarReading(
                Collections.unmodifiableList(detected),
                confidenceScore,
                timestamp
            );
        } catch (Exception e) {
            System.err.println("RadarSensor [" + sensorId + "]: malformed tick — "
                + e.getMessage());
            status = SensorStatus.FAILED;
            latestReading = null;
        }
    }

    /**
     * Opens the data file and streams only the metadata block.
     * Extracts weatherFactor and rfInterferenceFactor, then derives
     * confidenceScore via deriveConfidence(). Derives initial SensorStatus
     * from confidenceScore. Advances the reader past the ticks array opening
     * so the first readNextTickObject() call lands on tick index zero.
     *
     * The reader remains open after this call for use by readFrequencies().
     */
    private void openAndReadMetadata() {
        try {
            reader = new BufferedReader(new FileReader(dataFilePath));

            String metadataRaw = streamNextObject();
            if (metadataRaw == null) {
                System.err.println("RadarSensor [" + sensorId + "]: missing metadata block.");
                status = SensorStatus.FAILED;
                return;
            }

            JSONObject metadata          = new JSONObject(metadataRaw);
            float      weatherFactor     = (float) metadata.optDouble("weatherFactor", 1.0);
            float      rfFactor          = (float) metadata.optDouble("rfInterferenceFactor", 1.0);

            this.confidenceScore = deriveConfidence(weatherFactor, rfFactor);
            updateStatus(confidenceScore);

            advancePastTicksOpening();

        } catch (IOException e) {
            System.err.println("RadarSensor [" + sensorId + "]: failed to open "
                + dataFilePath + " — " + e.getMessage());
            status = SensorStatus.FAILED;
        }
    }

    /**
     * Derives a single confidenceScore from the two metadata factors.
     *
     * weatherFactor is first adjusted upward by WEATHER_RESISTANCE to reflect
     * radar's physical resilience to precipitation and fog relative to LiDAR
     * (FR2103). rfInterferenceFactor is applied without any resistance modifier
     * since frequency hopping (FR2108) is already represented by rfFactor not
     * reaching zero under normal interference conditions.
     *
     * confidenceScore = min(rfFactor, adjustedWeather)
     */
    private float deriveConfidence(float weatherFactor, float rfFactor) {
        float adjustedWeather = weatherFactor + (1.0f - weatherFactor) * WEATHER_RESISTANCE;
        return Math.min(rfFactor, adjustedWeather);
    }

    /**
     * Streams lines from the open reader until a complete JSON object is
     * accumulated (brace depth returns to zero after opening). Strips any
     * trailing comma left by JSON array formatting. Returns null if the end
     * of stream or a closing bracket is reached before an object starts.
     */
    private String streamNextObject() throws IOException {
        StringBuilder builder = new StringBuilder();
        String line;
        int braceDepth = 0;
        boolean started = false;

        while ((line = reader.readLine()) != null) {
            String trimmed = line.trim();
            if (trimmed.equals("]") || trimmed.equals("]}")) {
                return null;
            }
            if (trimmed.startsWith("{")) {
                started = true;
            }
            if (started) {
                builder.append(trimmed);
                for (char c : trimmed.toCharArray()) {
                    if (c == '{') braceDepth++;
                    if (c == '}') braceDepth--;
                }
                if (braceDepth == 0) {
                    String result = builder.toString();
                    if (result.endsWith(",")) {
                        result = result.substring(0, result.length() - 1);
                    }
                    return result;
                }
            }
        }
        return null;
    }

    /**
     * Advances the reader past the "ticks" key and its opening bracket so
     * that the next streamNextObject() call reads tick index zero.
     */
    private void advancePastTicksOpening() throws IOException {
        String line;
        while ((line = reader.readLine()) != null) {
            String trimmed = line.trim();
            if (trimmed.contains("\"ticks\"") || trimmed.equals("[")) {
                return;
            }
        }
    }

    /**
     * Calls streamNextObject() and wraps the checked IOException so that
     * readFrequencies() can remain free of try-catch for the common read path.
     */
    private String readNextTickObject() {
        try {
            return streamNextObject();
        } catch (IOException e) {
            System.err.println("RadarSensor [" + sensorId + "]: error reading tick — "
                + e.getMessage());
            return null;
        }
    }

    /**
     * Maps confidenceScore to SensorStatus.
     * FR2103: DEGRADED signals controller to increase LiDAR weighting discount.
     * FR2107: low confidence from RF interference triggers DEGRADED or FAILED.
     * FR2104: FAILED excludes this sensor from the 2oo3 vote.
     */
    private void updateStatus(float confidence) {
        if (confidence <= FAILED_THRESHOLD) {
            status = SensorStatus.FAILED;
        } else if (confidence <= DEGRADED_THRESHOLD) {
            status = SensorStatus.DEGRADED;
        } else {
            status = SensorStatus.OK;
        }
    }

    private void startClock() {
        clock = new Timer("RadarSensor-" + sensorId, true);
        clock.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                readFrequencies();
            }
        }, 0, CLOCK_INTERVAL_MS);
    }

    private void closeReader() {
        if (reader != null) {
            try {
                reader.close();
            } catch (IOException e) {
                System.err.println("RadarSensor [" + sensorId + "]: error closing reader — "
                    + e.getMessage());
            }
        }
    }
}