package Simulator.ScenarioGenerator;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * Generates all sensor JSON files for a ScenarioConfig.
 *
 * Outputs (all written to outputDirectory):
 *
 *   Camera  (50ms ticks, 3 copies):
 *     1worldCameraData.json, 2worldCameraData.json, 3worldCameraData.json
 *
 *   LiDAR   (100ms ticks, 3 copies):
 *     1worldLidarData.json, 2worldLidarData.json, 3worldLidarData.json
 *
 *   Radar   (100ms ticks, 3 copies):
 *     1worldRadarData.json, 2worldRadarData.json, 3worldRadarData.json
 *
 *   Wheel   (10ms ticks, 3 copies):
 *     worldWheelSpeedId1.json, worldWheelSpeedId2.json, worldWheelSpeedId3.json
 *
 *   Driver  (250ms ticks, 1 file):
 *     worldDriverData.json
 *
 * All files are generated from the same internal physics simulation, ensuring
 * consistency across sensors: if the wheel sensor shows 800 RPM, the radar and
 * LiDAR will show relative speed proportional to that vehicle speed.
 *
 * Usage:
 * <pre>
 *   ScenarioConfig cfg = new ScenarioConfig.Builder(...)
 *       .vehicleSpeedKmh(100)
 *       .addWorldObject(new WorldObject("truck1", ObjectClass.VEHICLE, 80, 0, 0))
 *       .addEvent(new ScenarioEvent(5.0, Type.EMERGENCY_BRAKE, 8.0))
 *       .build();
 *
 *   ScenarioGenerator gen = new ScenarioGenerator(cfg, "./scenarios/SC-001");
 *   gen.generate();
 * </pre>
 */
public class ScenarioGenerator {

    // -------------------------------------------------------------------------
    // Tick intervals (must match sensor implementations)
    // -------------------------------------------------------------------------

    private static final int CAMERA_TICK_MS = 50;
    private static final int LIDAR_TICK_MS  = 100;
    private static final int RADAR_TICK_MS  = 100;
    private static final int WHEEL_TICK_MS  = 10;
    private static final int DRIVER_TICK_MS = 250;

    private static final int SENSOR_COPIES  = 3;

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    private final ScenarioConfig config;
    private final String         outputDirectory;
    private final Random         rng;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param config          the fully-built scenario configuration
     * @param outputDirectory directory where JSON files will be written
     */
    public ScenarioGenerator(final ScenarioConfig config, final String outputDirectory) {
        assert config != null           : "config must not be null";
        assert outputDirectory != null  : "outputDirectory must not be null";
        this.config          = config;
        this.outputDirectory = outputDirectory;
        this.rng             = new Random(42L); // fixed seed for reproducibility
    }

    // -------------------------------------------------------------------------
    // Main entry point
    // -------------------------------------------------------------------------

    /**
     * Runs the scenario simulation and writes all sensor JSON files.
     *
     * @throws IOException if any file cannot be written
     */
    public void generate() throws IOException {
        System.out.println("ScenarioGenerator: generating scenario '"
            + config.scenarioId + "' → " + outputDirectory);

        // Ensure output directory exists
        new java.io.File(outputDirectory).mkdirs();

        // Determine total ticks for each sensor type
        final long totalMs = (long)(config.durationSeconds * 1000);

        // Build sorted event list
        final List<ScenarioEvent> sortedEvents = new ArrayList<>(config.events);
        sortedEvents.sort((a, b) -> Double.compare(a.triggerTimeSec, b.triggerTimeSec));

        // Deep-copy world objects so we can mutate them during simulation
        final List<WorldObject> worldObjects = deepCopyWorldObjects(config.worldObjects);

        // Run the master simulation loop — collect tick snapshots
        final List<TickSnapshot> snapshots = simulate(totalMs, sortedEvents, worldObjects);

        // Write all sensor files from the shared snapshots
        writeCameraFiles(snapshots);
        writeLidarFiles(snapshots);
        writeRadarFiles(snapshots);
        writeWheelFiles(snapshots);
        writeDriverFile(snapshots);

        System.out.println("ScenarioGenerator: complete. Files written to: " + outputDirectory);
    }

    // -------------------------------------------------------------------------
    // Master simulation loop
    // -------------------------------------------------------------------------

    /**
     * Runs the physics simulation at WHEEL_TICK_MS resolution (10ms) — the
     * finest-grained sensor. All coarser sensor ticks (50ms, 100ms, 250ms)
     * are sampled from this master timeline.
     *
     * At each tick:
     *   1. Apply any events whose triggerTimeSec falls within this tick.
     *   2. Advance vehicle physics.
     *   3. Advance world object physics.
     *   4. Record a TickSnapshot.
     *
     * @return list of snapshots, one per 10ms tick
     */
    private List<TickSnapshot> simulate(final long totalMs,
                                         final List<ScenarioEvent> events,
                                         final List<WorldObject> worldObjects) {
        final List<TickSnapshot> snapshots = new ArrayList<>();
        final VehicleState vehicle = new VehicleState(config.vehicleSpeedKmh);

        final double dtSec = WHEEL_TICK_MS / 1000.0;
        int eventIdx = 0;

        for (long t = 0; t <= totalMs; t += WHEEL_TICK_MS) {
            final double timeSec = t / 1000.0;

            // 1. Apply events
            while (eventIdx < events.size()
                   && events.get(eventIdx).triggerTimeSec <= timeSec) {
                applyEvent(events.get(eventIdx), vehicle, worldObjects);
                eventIdx++;
            }

            // 2. Advance vehicle physics
            final double distAdvanced = vehicle.advance(dtSec);

            // 3. Advance world objects
            for (final WorldObject obj : worldObjects) {
                obj.advanceOwnMotion(dtSec);
                obj.adjustForEgoMotion(distAdvanced);

                if (vehicle.getTurnRateDegPerSec() != 0.0) {
                    final double turnDelta = vehicle.getTurnRateDegPerSec() * dtSec;
                    obj.adjustForEgoTurn(turnDelta);
                }
            }

            // 4. Record snapshot
            final float[] wheelRpm = vehicle.getWheelRpm(config.sensorNoiseLevel, rng);
            final boolean driverBraking = isDriverBraking(timeSec);

            snapshots.add(new TickSnapshot(
                t,
                timeSec,
                vehicle.getSpeedKmh(),
                vehicle.isBraking(),
                wheelRpm,
                driverBraking,
                snapshotObjects(worldObjects, timeSec, vehicle.getSpeedKmh())
            ));
        }

        return snapshots;
    }

    // -------------------------------------------------------------------------
    // Event application
    // -------------------------------------------------------------------------

    private void applyEvent(final ScenarioEvent event,
                             final VehicleState vehicle,
                             final List<WorldObject> worldObjects) {
        System.out.printf("  [t=%.3fs] Applying event: %s%n",
            event.triggerTimeSec, event.type);

        switch (event.type) {
            case TURN_RIGHT:
                vehicle.setTurnRate(Math.abs(event.param1));
                break;

            case TURN_LEFT:
                vehicle.setTurnRate(-Math.abs(event.param1));
                break;

            case TURN_END:
                vehicle.setTurnRate(0.0);
                break;

            case EMERGENCY_BRAKE:
                vehicle.setAcceleration(-Math.abs(event.param1));
                break;

            case NORMAL_BRAKE:
                vehicle.setAcceleration(-Math.abs(event.param1));
                break;

            case SET_SPEED:
                vehicle.setSpeed(event.param1);
                break;

            case ACCELERATE:
                vehicle.setAcceleration(Math.abs(event.param1), event.param2);
                break;

            case COAST:
                vehicle.coast();
                break;

            case OBJECT_APPEAR:
                findObject(worldObjects, event.stringParam)
                    .ifPresent(obj -> obj.setVisible(true));
                break;

            case OBJECT_DISAPPEAR:
                findObject(worldObjects, event.stringParam)
                    .ifPresent(obj -> obj.setVisible(false));
                break;

            case OBJECT_SET_SPEED:
                findObject(worldObjects, event.stringParam)
                    .ifPresent(obj -> obj.setObjectSpeedKmh(event.param1));
                break;

            case OBJECT_SET_HEADING:
                findObject(worldObjects, event.stringParam)
                    .ifPresent(obj -> obj.setHeadingDegrees(event.param1));
                break;

            case WEATHER_CHANGE:
            case SENSOR_DROPOUT:
                // Documented in comments; sensor confidence is set at metadata time.
                System.out.println("    (informational event — noted in log)");
                break;

            default:
                System.err.println("    Unknown event type: " + event.type);
        }
    }

    // -------------------------------------------------------------------------
    // File writers
    // -------------------------------------------------------------------------

    /**
     * Writes 3 camera JSON files.
     * Camera ticks every 50ms.
     * Format: metadata block with ticks array containing objectType + bearingDegrees.
     *
     * Sensor 1 gets perfect data; sensors 2 and 3 get small independent jitter
     * to simulate physically separate lenses seeing slightly different views —
     * the CameraVoter 2oo3 logic requires this to be meaningful.
     */
    private void writeCameraFiles(final List<TickSnapshot> snapshots) throws IOException {
        final long tickStep = CAMERA_TICK_MS / WHEEL_TICK_MS; // =5

        for (int sensorId = 1; sensorId <= SENSOR_COPIES; sensorId++) {
            final String filename = outputDirectory + "/" + sensorId + "worldCameraData.json";
            try (final PrintWriter w = openWriter(filename)) {
                writeMetadataOpen(w, config.scenarioId, config.description, config.weatherFactor);
                w.println("  \"ticks\": [");

                boolean first = true;
                for (long i = 0; i < snapshots.size(); i += tickStep) {
                    final TickSnapshot snap = snapshots.get((int) i);

                    if (!first) { w.println(","); }
                    first = false;

                    w.printf("    {%n");
                    w.printf("      \"timestampMs\": %d,%n", snap.timestampMs);
                    w.printf("      \"detectedObjects\": [");

                    boolean firstObj = true;
                    for (final ObjectSnapshot obj : snap.objects) {
                        if (!obj.visible) { continue; }
                        if (!firstObj) { w.print(", "); }
                        firstObj = false;

                        // Small per-sensor bearing jitter to differentiate cameras
                        final double bearingJitter = (sensorId - 1) * 0.1
                            + (rng.nextDouble() - 0.5) * config.sensorNoiseLevel * 2.0;
                        final double bearing = obj.bearingDegrees + bearingJitter;

                        w.printf("%n        { \"objectType\": \"%s\", \"bearingDegrees\": %.2f }",
                            obj.cameraObjectType, bearing);
                    }

                    if (!firstObj) { w.println(); w.print("      ]"); }
                    else           { w.print("]"); }
                    w.println();
                    w.print("    }");
                }

                w.println();
                w.println("  ]");
                w.println("}");
            }
            System.out.println("  Written: " + filename);
        }
    }

    /**
     * Writes 3 LiDAR JSON files.
     * LiDAR ticks every 100ms.
     * Format: metadata with weatherFactor, ticks array with detectedObjects
     * containing distanceMetres, relativeSpeedKmh, bearingDegrees.
     */
    private void writeLidarFiles(final List<TickSnapshot> snapshots) throws IOException {
        final long tickStep = LIDAR_TICK_MS / WHEEL_TICK_MS; // =10

        for (int sensorId = 1; sensorId <= SENSOR_COPIES; sensorId++) {
            final String filename = outputDirectory + "/" + sensorId + "worldLidarData.json";
            try (final PrintWriter w = openWriter(filename)) {
                // LiDAR metadata: weatherFactor directly (no rf factor)
                writeLidarRadarMetadataOpen(w, config.weatherFactor, -1f);
                w.println("  \"ticks\": [");

                boolean first = true;
                for (long i = 0; i < snapshots.size(); i += tickStep) {
                    final TickSnapshot snap = snapshots.get((int) i);

                    if (!first) { w.println(","); }
                    first = false;

                    w.printf("    {%n");
                    w.printf("      \"timestampMs\": %d,%n", snap.timestampMs);
                    w.printf("      \"detectedObjects\": [");

                    boolean firstObj = true;
                    for (final ObjectSnapshot obj : snap.objects) {
                        if (!obj.visible) { continue; }
                        if (!firstObj) { w.print(", "); }
                        firstObj = false;

                        // Per-sensor independent measurement noise
                        final double distNoise    = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * obj.distanceMetres * 0.02;
                        final double bearingNoise = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * 0.5;
                        final double speedNoise   = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * 1.0;

                        w.printf("%n        { \"distanceMetres\": %.2f, "
                                + "\"relativeSpeedKmh\": %.2f, "
                                + "\"bearingDegrees\": %.2f }",
                            Math.max(0.1, obj.distanceMetres + distNoise),
                            obj.relativeSpeedKmh + speedNoise,
                            obj.bearingDegrees + bearingNoise);
                    }

                    if (!firstObj) { w.println(); w.print("      ]"); }
                    else           { w.print("]"); }
                    w.println();
                    w.print("    }");
                }

                w.println();
                w.println("  ]");
                w.println("}");
            }
            System.out.println("  Written: " + filename);
        }
    }

    /**
     * Writes 3 Radar JSON files.
     * Radar ticks every 100ms.
     * Same shape as LiDAR but metadata includes rfInterferenceFactor.
     * Radar also has better distance accuracy but slightly wider bearing spread.
     */
    private void writeRadarFiles(final List<TickSnapshot> snapshots) throws IOException {
        final long tickStep = RADAR_TICK_MS / WHEEL_TICK_MS; // =10

        for (int sensorId = 1; sensorId <= SENSOR_COPIES; sensorId++) {
            final String filename = outputDirectory + "/" + sensorId + "worldRadarData.json";
            try (final PrintWriter w = openWriter(filename)) {
                writeLidarRadarMetadataOpen(w, config.weatherFactor, config.rfInterferenceFactor);
                w.println("  \"ticks\": [");

                boolean first = true;
                for (long i = 0; i < snapshots.size(); i += tickStep) {
                    final TickSnapshot snap = snapshots.get((int) i);

                    if (!first) { w.println(","); }
                    first = false;

                    w.printf("    {%n");
                    w.printf("      \"timestampMs\": %d,%n", snap.timestampMs);
                    w.printf("      \"detectedObjects\": [");

                    boolean firstObj = true;
                    for (final ObjectSnapshot obj : snap.objects) {
                        if (!obj.visible) { continue; }
                        if (!firstObj) { w.print(", "); }
                        firstObj = false;

                        // Radar has tighter distance noise but wider bearing noise
                        final double distNoise    = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * obj.distanceMetres * 0.01;
                        final double bearingNoise = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * 1.5;
                        final double speedNoise   = (rng.nextDouble() - 0.5)
                            * config.sensorNoiseLevel * 0.5;

                        w.printf("%n        { \"distanceMetres\": %.2f, "
                                + "\"relativeSpeedKmh\": %.2f, "
                                + "\"bearingDegrees\": %.2f }",
                            Math.max(0.1, obj.distanceMetres + distNoise),
                            obj.relativeSpeedKmh + speedNoise,
                            obj.bearingDegrees + bearingNoise);
                    }

                    if (!firstObj) { w.println(); w.print("      ]"); }
                    else           { w.print("]"); }
                    w.println();
                    w.print("    }");
                }

                w.println();
                w.println("  ]");
                w.println("}");
            }
            System.out.println("  Written: " + filename);
        }
    }

    /**
     * Writes 3 Wheel sensor JSON files.
     * Wheel sensor ticks every 10ms (master resolution — no sub-sampling needed).
     * Format uses "frames" array (not "ticks") with rpm[4] per the WheelSensor reader.
     */
    private void writeWheelFiles(final List<TickSnapshot> snapshots) throws IOException {
        for (int sensorId = 1; sensorId <= SENSOR_COPIES; sensorId++) {
            final String filename = outputDirectory + "/worldWheelSpeedId" + sensorId + ".json";
            try (final PrintWriter w = openWriter(filename)) {
                w.println("{");
                w.printf("  \"metadata\": {%n");
                w.printf("    \"scenarioId\": \"%s\",%n", config.scenarioId);
                w.printf("    \"description\": \"%s\",%n", config.description);
                w.printf("    \"sensorId\": %d%n", sensorId);
                w.println("  },");
                w.println("  \"frames\": [");

                boolean first = true;
                for (final TickSnapshot snap : snapshots) {
                    if (!first) { w.println(","); }
                    first = false;

                    // Each sensor gets slightly different RPM noise (independent jitter)
                    final float noiseScale = 1.0f + (sensorId - 1) * 0.001f;
                    final float fl = snap.wheelRpm[0] * noiseScale;
                    final float fr = snap.wheelRpm[1] * noiseScale;
                    final float rl = snap.wheelRpm[2] * noiseScale;
                    final float rr = snap.wheelRpm[3] * noiseScale;

                    w.printf("    { \"timestampMs\": %d, \"rpm\": [%.2f, %.2f, %.2f, %.2f] }",
                        snap.timestampMs, fl, fr, rl, rr);
                }

                w.println();
                w.println("  ]");
                w.println("}");
            }
            System.out.println("  Written: " + filename);
        }
    }

    /**
     * Writes the single driver scenario JSON file.
     * Driver ticks every 250ms.
     * Format: metadata block + ticks array with timestampMs and action (BRAKE/NONE).
     */
    private void writeDriverFile(final List<TickSnapshot> snapshots) throws IOException {
        final long tickStep = DRIVER_TICK_MS / WHEEL_TICK_MS; // =25

        final String filename = outputDirectory + "/worldDriverData.json";
        try (final PrintWriter w = openWriter(filename)) {
            w.println("{");
            w.printf("  \"metadata\": {%n");
            w.printf("    \"scenarioId\": \"%s\",%n", config.scenarioId);
            w.printf("    \"description\": \"%s\",%n", config.description);
            w.printf("    \"driverBrakes\": %b,%n", config.driverBrakes);
            w.printf("    \"driverReactionTimeSec\": %.2f%n", config.driverReactionTimeSec);
            w.println("  },");
            w.println("  \"ticks\": [");

            boolean first = true;
            for (long i = 0; i < snapshots.size(); i += tickStep) {
                final TickSnapshot snap = snapshots.get((int) i);

                if (!first) { w.println(","); }
                first = false;

                final String action = snap.driverBraking ? "BRAKE" : "NONE";
                w.printf("    { \"timestampMs\": %d, \"action\": \"%s\" }",
                    snap.timestampMs, action);
            }

            w.println();
            w.println("  ]");
            w.println("}");
        }
        System.out.println("  Written: " + filename);
    }

    // -------------------------------------------------------------------------
    // Metadata helpers
    // -------------------------------------------------------------------------

    /**
     * Writes the opening metadata block used by Camera files.
     * Camera reads metadata as an outer JSON object containing a "metadata" key.
     */
    private void writeMetadataOpen(final PrintWriter w,
                                    final String scenarioId,
                                    final String description,
                                    final float weatherFactor) {
        w.println("{");
        w.println("  \"metadata\": {");
        w.printf("    \"scenarioId\": \"%s\",%n", scenarioId);
        w.printf("    \"description\": \"%s\",%n", description);
        w.printf("    \"weatherFactor\": %.2f%n", weatherFactor);
        w.println("  },");
    }

    /**
     * Writes the opening metadata block used by LiDAR and Radar files.
     * These sensors read a flat metadata object (not nested under a "metadata" key).
     *
     * @param rfFactor  -1 to omit rfInterferenceFactor (LiDAR), otherwise include it
     */
    private void writeLidarRadarMetadataOpen(final PrintWriter w,
                                              final float weatherFactor,
                                              final float rfFactor) {
        w.println("{");
        w.println("  \"metadata\": {");
        w.printf("    \"weatherFactor\": %.2f", weatherFactor);
        if (rfFactor >= 0) {
            w.println(",");
            w.printf("    \"rfInterferenceFactor\": %.2f%n", rfFactor);
        } else {
            w.println();
        }
        w.println("  },");
    }

    // -------------------------------------------------------------------------
    // Snapshot helpers
    // -------------------------------------------------------------------------

    /**
     * Creates immutable snapshots of all world objects at the current tick.
     */
    private List<ObjectSnapshot> snapshotObjects(final List<WorldObject> objects,
                                                   final double timeSec,
                                                   final double egoSpeedKmh) {
        final List<ObjectSnapshot> result = new ArrayList<>();
        for (final WorldObject obj : objects) {
            final boolean visible = obj.isVisible(timeSec);
            final double dist     = obj.getDistanceMetres();
            final double bearing  = obj.getBearingDegrees();
            final double relSpeed = obj.getRelativeSpeedKmh(egoSpeedKmh);

            result.add(new ObjectSnapshot(
                obj.id,
                visible,
                obj.toCameraObjectType(),
                dist,
                relSpeed,
                bearing
            ));
        }
        return result;
    }

    // -------------------------------------------------------------------------
    // Driver braking logic
    // -------------------------------------------------------------------------

    /**
     * Returns true if the driver should be braking at the given time.
     *
     * Driver brakes when:
     *   - driverBrakes = true in config
     *   - currentTimeSec >= driverReactionTimeSec
     */
    private boolean isDriverBraking(final double currentTimeSec) {
        if (!config.driverBrakes) { return false; }
        return currentTimeSec >= config.driverReactionTimeSec;
    }

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    private PrintWriter openWriter(final String path) throws IOException {
        return new PrintWriter(new BufferedWriter(new FileWriter(path)));
    }

    /**
     * Deep-copies world objects so simulation mutations don't affect the
     * original config (which uses an immutable List).
     */
    private List<WorldObject> deepCopyWorldObjects(final List<WorldObject> originals) {
        final List<WorldObject> copies = new ArrayList<>();
        for (final WorldObject o : originals) {
            copies.add(new WorldObject(
                o.id,
                o.objectClass,
                o.getRawDistanceAhead(),
                o.getRawLateralOffset(),
                o.getObjectSpeedKmh(),
                o.getHeadingDegrees(),
                o.appearsAtSec,
                o.disappearsAtSec
            ));
        }
        return copies;
    }

    private java.util.Optional<WorldObject> findObject(final List<WorldObject> objects,
                                                        final String id) {
        return objects.stream().filter(o -> o.id.equals(id)).findFirst();
    }

    // -------------------------------------------------------------------------
    // Inner records — package-private for testability
    // -------------------------------------------------------------------------

    /**
     * Immutable snapshot of the entire world state at one 10ms master tick.
     */
    static final class TickSnapshot {
        final long               timestampMs;
        final double             timeSec;
        final double             egoSpeedKmh;
        final boolean            egoBraking;
        final float[]            wheelRpm;
        final boolean            driverBraking;
        final List<ObjectSnapshot> objects;

        TickSnapshot(final long timestampMs,
                     final double timeSec,
                     final double egoSpeedKmh,
                     final boolean egoBraking,
                     final float[] wheelRpm,
                     final boolean driverBraking,
                     final List<ObjectSnapshot> objects) {
            this.timestampMs   = timestampMs;
            this.timeSec       = timeSec;
            this.egoSpeedKmh   = egoSpeedKmh;
            this.egoBraking    = egoBraking;
            this.wheelRpm      = wheelRpm;
            this.driverBraking = driverBraking;
            this.objects       = Collections.unmodifiableList(new ArrayList<ObjectSnapshot>(objects));
        }
    }

    /**
     * Immutable snapshot of a single world object at one tick.
     */
    static final class ObjectSnapshot {
        final String  id;
        final boolean visible;
        final String  cameraObjectType;
        final double  distanceMetres;
        final double  relativeSpeedKmh;
        final double  bearingDegrees;

        ObjectSnapshot(final String id,
                       final boolean visible,
                       final String cameraObjectType,
                       final double distanceMetres,
                       final double relativeSpeedKmh,
                       final double bearingDegrees) {
            this.id               = id;
            this.visible          = visible;
            this.cameraObjectType = cameraObjectType;
            this.distanceMetres   = distanceMetres;
            this.relativeSpeedKmh = relativeSpeedKmh;
            this.bearingDegrees   = bearingDegrees;
        }
    }
}