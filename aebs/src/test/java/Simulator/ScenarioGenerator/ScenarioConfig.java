package Simulator.ScenarioGenerator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Immutable configuration for a simulated AEBS driving scenario.
 *
 * Passed to ScenarioGenerator to produce all sensor JSON files.
 * Build with ScenarioConfig.Builder.
 *
 * All distances in metres, speeds in km/h, angles in degrees.
 * Bearing convention: 0 = directly ahead, positive = right, negative = left.
 *
 * Example usage:
 * <pre>
 *   ScenarioConfig config = new ScenarioConfig.Builder("SC-002", "Highway near-miss", 10)
 *       .vehicleSpeedKmh(100)
 *       .weatherFactor(0.85f)
 *       .addWorldObject(new WorldObject("truck1", ObjectClass.VEHICLE, 80f, 0f, 0f))
 *       .addEvent(new ScenarioEvent(5.0, ScenarioEvent.Type.EMERGENCY_BRAKE, 0))
 *       .build();
 * </pre>
 */
public final class ScenarioConfig {

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Unique identifier for this scenario */
    public final String scenarioId;

    /** Human-readable description written into metadata blocks */
    public final String description;

    /** Total scenario duration in seconds */
    public final double durationSeconds;

    /** Initial vehicle speed in km/h */
    public final double vehicleSpeedKmh;

    /**
     * Weather factor: 1.0 = clear, 0.0 = sensor-blind.
     * Applied to Camera confidence and LiDAR confidence directly.
     * Radar applies its built-in weather resistance on top of this.
     */
    public final float weatherFactor;

    /**
     * RF interference factor for radar: 1.0 = no interference, 0.0 = jammed.
     * Does not affect Camera or LiDAR.
     */
    public final float rfInterferenceFactor;

    /** Objects present in the world at the start of the scenario */
    public final List<WorldObject> worldObjects;

    /** Timed events that alter vehicle or world state during the scenario */
    public final List<ScenarioEvent> events;

    /**
     * Noise level applied to sensor readings: 0.0 = perfect, 1.0 = maximum noise.
     * Models small per-tick jitter in distance/bearing measurements.
     */
    public final float sensorNoiseLevel;

    /**
     * Whether the driver brakes in response to the primary hazard.
     * If false, driver action stays NONE throughout (inattentive driver).
     * If true, driver brakes at driverReactionTimeSec.
     */
    public final boolean driverBrakes;

    /** Time in seconds after hazard detection that the driver starts braking */
    public final double driverReactionTimeSec;

    // -------------------------------------------------------------------------
    // Constructor (private - use Builder)
    // -------------------------------------------------------------------------

    private ScenarioConfig(final Builder b) {
        this.scenarioId            = b.scenarioId;
        this.description           = b.description;
        this.durationSeconds       = b.durationSeconds;
        this.vehicleSpeedKmh       = b.vehicleSpeedKmh;
        this.weatherFactor         = b.weatherFactor;
        this.rfInterferenceFactor  = b.rfInterferenceFactor;
        this.worldObjects          = Collections.unmodifiableList(new ArrayList<WorldObject>(b.worldObjects));
        this.events                = Collections.unmodifiableList(new ArrayList<ScenarioEvent>(b.events));
        this.sensorNoiseLevel      = b.sensorNoiseLevel;
        this.driverBrakes          = b.driverBrakes;
        this.driverReactionTimeSec = b.driverReactionTimeSec;
    }

    // -------------------------------------------------------------------------
    // Builder
    // -------------------------------------------------------------------------

    public static final class Builder {
        private final String scenarioId;
        private final String description;
        private final double durationSeconds;

        private double vehicleSpeedKmh      = 60.0;
        private float  weatherFactor        = 1.0f;
        private float  rfInterferenceFactor = 1.0f;
        private float  sensorNoiseLevel     = 0.02f;
        private boolean driverBrakes        = true;
        private double driverReactionTimeSec = 1.5;

        private final List<WorldObject>    worldObjects = new ArrayList<>();
        private final List<ScenarioEvent>  events       = new ArrayList<>();

        /**
         * @param scenarioId      unique ID, e.g. "SC-003"
         * @param description     human-readable label for metadata blocks
         * @param durationSeconds total scenario runtime
         */
        public Builder(final String scenarioId,
                       final String description,
                       final double durationSeconds) {
            this.scenarioId      = scenarioId;
            this.description     = description;
            this.durationSeconds = durationSeconds;
        }

        /** Initial vehicle speed. Default: 60 km/h */
        public Builder vehicleSpeedKmh(final double kmh) {
            this.vehicleSpeedKmh = kmh;
            return this;
        }

        /**
         * Weather factor: 1.0 = clear, 0.0 = sensor-blind.
         * Default: 1.0 (clear)
         */
        public Builder weatherFactor(final float factor) {
            this.weatherFactor = Math.max(0f, Math.min(1f, factor));
            return this;
        }

        /**
         * RF interference factor for radar only. Default: 1.0 (no interference)
         */
        public Builder rfInterferenceFactor(final float factor) {
            this.rfInterferenceFactor = Math.max(0f, Math.min(1f, factor));
            return this;
        }

        /**
         * Sensor noise level: jitter applied to measurements. Default: 0.02
         */
        public Builder sensorNoiseLevel(final float noise) {
            this.sensorNoiseLevel = Math.max(0f, Math.min(1f, noise));
            return this;
        }

        /**
         * Whether the driver brakes in response to hazards.
         * Default: true (driver brakes after reactionTime)
         */
        public Builder driverBrakes(final boolean brakes) {
            this.driverBrakes = brakes;
            return this;
        }

        /**
         * Time in seconds from scenario start that the driver starts braking.
         * Only relevant if driverBrakes = true. Default: 1.5s
         */
        public Builder driverReactionTimeSec(final double seconds) {
            this.driverReactionTimeSec = seconds;
            return this;
        }

        /** Adds a world object to the scene */
        public Builder addWorldObject(final WorldObject obj) {
            this.worldObjects.add(obj);
            return this;
        }

        /** Adds a timed scenario event */
        public Builder addEvent(final ScenarioEvent event) {
            this.events.add(event);
            return this;
        }

        public ScenarioConfig build() {
            return new ScenarioConfig(this);
        }
    }
}