package Simulator.ScenarioGenerator;

/**
 * A timed event that modifies vehicle or world state during a scenario.
 *
 * Events are processed by ScenarioGenerator at the appropriate simulation
 * tick and alter the internal physics state used for subsequent ticks.
 * All bearing and distance values reported by sensors after an event will
 * reflect the changed state automatically.
 *
 * Example events:
 *   TURN_RIGHT at 10s — ego vehicle begins turning right; object bearings
 *                        shift left relative to the new heading.
 *   EMERGENCY_BRAKE at 5s — ego vehicle decelerates at 8 m/s².
 *   OBJECT_APPEAR at 3s — a hidden WorldObject becomes visible to sensors.
 *
 * Events are applied instantaneously at their trigger time.
 * Smooth transitions (e.g. gradual turns) are modelled by multiple
 * small TURN events, or by using TURN_START / TURN_END pairs.
 */
public class ScenarioEvent {

    // -------------------------------------------------------------------------
    // Event types
    // -------------------------------------------------------------------------

    public enum Type {
        /**
         * Ego vehicle begins turning right.
         * param1 = turn rate in degrees per second (positive).
         * param2 = duration of turn in seconds (0 = instantaneous snap).
         */
        TURN_RIGHT,

        /**
         * Ego vehicle begins turning left.
         * param1 = turn rate in degrees per second (positive).
         * param2 = duration of turn in seconds (0 = instantaneous snap).
         */
        TURN_LEFT,

        /**
         * Ego vehicle stops turning — restores turn rate to 0.
         * No params required.
         */
        TURN_END,

        /**
         * Ego vehicle applies emergency braking.
         * param1 = deceleration in m/s² (positive value, e.g. 8.0).
         * param2 = unused.
         * Braking continues until vehicle stops or a SET_SPEED event overrides.
         */
        EMERGENCY_BRAKE,

        /**
         * Ego vehicle applies normal braking.
         * param1 = deceleration in m/s² (e.g. 3.0).
         * param2 = unused.
         */
        NORMAL_BRAKE,

        /**
         * Sets the ego vehicle speed directly (no ramp).
         * param1 = new speed in km/h.
         * param2 = unused.
         * Use this to resume speed after a brake event.
         */
        SET_SPEED,

        /**
         * Sets the ego vehicle acceleration (for speeding up scenarios).
         * param1 = acceleration in m/s².
         * param2 = max speed cap in km/h (0 = no cap).
         */
        ACCELERATE,

        /**
         * Stops acceleration or deceleration — coast at current speed.
         */
        COAST,

        /**
         * A named WorldObject becomes visible to sensors.
         * stringParam = WorldObject id.
         * param1 = unused, param2 = unused.
         */
        OBJECT_APPEAR,

        /**
         * A named WorldObject disappears from sensors (e.g. pedestrian steps back).
         * stringParam = WorldObject id.
         */
        OBJECT_DISAPPEAR,

        /**
         * Changes a named WorldObject's speed.
         * stringParam = WorldObject id.
         * param1 = new speed in km/h.
         * param2 = unused.
         */
        OBJECT_SET_SPEED,

        /**
         * Changes a named WorldObject's heading.
         * stringParam = WorldObject id.
         * param1 = new heading in degrees (0 = same as ego).
         */
        OBJECT_SET_HEADING,

        /**
         * Simulates weather worsening mid-scenario (e.g. rain starts).
         * param1 = new weatherFactor (0.0–1.0).
         * Note: metadata weatherFactor is fixed at scenario start per sensor design.
         * This event records the change but sensor confidence doesn't retroactively change.
         * It is recorded in a comment in generated JSON for documentation.
         */
        WEATHER_CHANGE,

        /**
         * Sensor dropout event — marks one sensor instance as unavailable for a period.
         * stringParam = sensor type prefix, e.g. "CAMERA_1", "LIDAR_2", "RADAR_3".
         * param1 = duration in seconds of the dropout (0 = permanent failure).
         */
        SENSOR_DROPOUT
    }

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Time in seconds from scenario start when this event triggers */
    public final double triggerTimeSec;

    /** The type of event */
    public final Type type;

    /** Primary numeric parameter (semantics depend on type) */
    public final double param1;

    /** Secondary numeric parameter (semantics depend on type) */
    public final double param2;

    /** String parameter used for object-targeting events */
    public final String stringParam;

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /**
     * Creates an event with numeric parameters.
     *
     * @param triggerTimeSec  seconds from scenario start
     * @param type            event type
     * @param param1          primary parameter (see type javadoc)
     */
    public ScenarioEvent(final double triggerTimeSec,
                         final Type type,
                         final double param1) {
        this(triggerTimeSec, type, param1, 0.0, null);
    }

    /**
     * Creates an event with two numeric parameters.
     */
    public ScenarioEvent(final double triggerTimeSec,
                         final Type type,
                         final double param1,
                         final double param2) {
        this(triggerTimeSec, type, param1, param2, null);
    }

    /**
     * Creates an event targeting a named world object.
     *
     * @param triggerTimeSec  seconds from scenario start
     * @param type            event type
     * @param stringParam     id of the target WorldObject
     * @param param1          primary parameter
     */
    public ScenarioEvent(final double triggerTimeSec,
                         final Type type,
                         final String stringParam,
                         final double param1) {
        this(triggerTimeSec, type, param1, 0.0, stringParam);
    }

    /**
     * Full constructor.
     */
    public ScenarioEvent(final double triggerTimeSec,
                         final Type type,
                         final double param1,
                         final double param2,
                         final String stringParam) {
        this.triggerTimeSec = triggerTimeSec;
        this.type           = type;
        this.param1         = param1;
        this.param2         = param2;
        this.stringParam    = stringParam;
    }
}
