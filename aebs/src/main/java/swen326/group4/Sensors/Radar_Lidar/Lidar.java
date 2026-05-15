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
 * LiDAR sensor implementation.
 *
 * On start(), openAndReadMetadata() opens worldLidarData.json and streams
 * only the metadata block to extract the scenario-wide weatherFactor.
 * This is used directly as the confidenceScore for all readings this sensor
 * produces — LiDAR applies no weather resistance modifier, making it more
 * sensitive to adverse conditions than radar (FR2103).
 *
 * The file is held open after metadata is consumed. On each 100ms clock
 * tick, castRays() streams only the next tick entry from the file and
 * produces a RadarLidarReading stamped with the scenario-wide confidenceScore.
 * No more than one tick is held in memory at any time.
 *
 * Confidence derivation for LiDAR:
 *   confidenceScore = weatherFactor
 *   where 0.0 = sensor blind, 1.0 = full visibility.
 *
 * Requirement traceability:
 *   FR2103 : DEGRADED status signals controller to shift braking weight to radar.
 *   FR2104 : FAILED status excludes this sensor from the 2oo3 vote entirely.
 */
public class Lidar implements RadarLidarSensor {

    private static final int    CLOCK_INTERVAL_MS  = 100;
    private static final float  DEGRADED_THRESHOLD = 0.5f;
    private static final float  FAILED_THRESHOLD   = 0.1f;
    private static final String DATA_FILE          = "worldLidarData.json";

    private final String      sensorId;
    private final String      dataFilePath;

    private float             confidenceScore;
    private BufferedReader    reader;
    private boolean           ticksExhausted;

    private Timer             clock;
    private RadarLidarReading latestReading;
    private SensorStatus      status;

    public Lidar(String sensorId, String dataDirectory) {
        this.sensorId        = sensorId;
        this.dataFilePath    = dataDirectory + "/" + sensorId + DATA_FILE;
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
     * weatherFactor is a scenario-wide constant read once from metadata.
     *
     * Sets status FAILED and nulls latestReading if ticks are exhausted
     * or the tick entry cannot be parsed (FR2104).
     */
    private void castRays() {
        
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

            System.out.println(
                latestReading.detectedObjects()
                + " \n " +
                latestReading.confidenceScore()
                + " \n " +
                latestReading.timestampMs()
                + " \n "
            );
        } catch (Exception e) {
            System.err.println("LidarSensor [" + sensorId + "]: malformed tick — "
                + e.getMessage());
            status = SensorStatus.FAILED;
            latestReading = null;
        }
    }

    /**
     * Opens the data file and streams only the metadata block.
     * Extracts weatherFactor and sets confidenceScore = weatherFactor.
     * LiDAR confidence maps directly to weatherFactor with no modifier.
     * Derives initial SensorStatus from confidenceScore.
     * Advances the reader past the ticks array opening so that the first
     * readNextTickObject() call lands on tick index zero.
     *
     * The reader remains open after this call for use by castRays().
     */
    private void openAndReadMetadata() {
        try {
            reader = new BufferedReader(new FileReader(dataFilePath));
            
            // String line = reader.readLine();
            // if (!line.equals("{")) {
            //     System.out.println("Missing starting brace");
            // }

            //System.out.println("Retrieveing metadata for LiDAR " + sensorId);

            String metadataRaw = streamNextObject();

           

            if (metadataRaw == null) {
                System.err.println("LidarSensor [" + sensorId + "]: missing metadata block.");
                status = SensorStatus.FAILED;
                return;
            }

            JSONObject metadata  = new JSONObject(metadataRaw);
            float      weatherFactor = (float) metadata.optDouble("weatherFactor", 1.0);

            this.confidenceScore = weatherFactor;
            updateStatus(confidenceScore);

            advancePastTicksOpening();

        } catch (IOException e) {
            System.err.println("LidarSensor [" + sensorId + "]: failed to open "
                + dataFilePath + " — " + e.getMessage());
            status = SensorStatus.FAILED;
        }
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
        boolean metadata = false;
        while ((line = reader.readLine()) != null) {
            String trimmed = line.trim();
            //System.out.println(trimmed);

            if (trimmed.contains("metadata")){
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
                //System.out.println(trimmed);
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
                    if (metadata){
                        result += '}';
                    }
                    //System.out.println(result);
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
     * castRays() can remain free of try-catch for the common read path.
     */
    private String readNextTickObject() {
        try {
            return streamNextObject();
        } catch (IOException e) {
            System.err.println("LidarSensor [" + sensorId + "]: error reading tick — "
                + e.getMessage());
            return null;
        }
    }

    /**
     * Maps confidenceScore to SensorStatus.
     * For LiDAR, confidenceScore == weatherFactor.
     * FR2103: DEGRADED signals controller to increase radar weighting.
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
        clock = new Timer("LidarSensor-" + sensorId, false);
        clock.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                
                castRays();
            }
        }, 0, CLOCK_INTERVAL_MS);
        System.out.println("Timer started!");
    }

    private void closeReader() {
        if (reader != null) {
            try {
                reader.close();
            } catch (IOException e) {
                System.err.println("LidarSensor [" + sensorId + "]: error closing reader — "
                    + e.getMessage());
            }
        }
    }
}