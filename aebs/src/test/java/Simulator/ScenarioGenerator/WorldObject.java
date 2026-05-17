package Simulator.ScenarioGenerator;

/**
 * A persistent object in the simulated world.
 *
 * WorldObject tracks position relative to the ego vehicle, its own speed,
 * and heading. The generator updates its bearing and distance each tick
 * based on both the object's own motion and the ego vehicle's motion.
 *
 * Position convention (all relative to ego vehicle):
 *   distanceMetres  — positive = ahead, negative = behind
 *   bearingDegrees  — 0 = directly ahead, +right, -left
 *   objectSpeedKmh  — object's own absolute speed (0 = stationary)
 *   headingDegrees  — direction object is travelling, 0 = same as ego
 *
 * Camera sees: objectType, bearingDegrees
 * LiDAR/Radar see: distanceMetres, relativeSpeedKmh, bearingDegrees
 * Wheel sensor is vehicle-internal — not affected by world objects directly.
 */
public class WorldObject {

    // -------------------------------------------------------------------------
    // Object classification - matches Camera.ObjectType
    // -------------------------------------------------------------------------

    public enum ObjectClass {
        VEHICLE,
        PEDESTRIAN,
        CYCLIST,
        STATIONARY_OBJECT,
        UNKNOWN
    }

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Unique identifier for this object within the scenario */
    public final String id;

    /** Classification - determines camera objectType output */
    public final ObjectClass objectClass;

    /**
     * Distance ahead of the ego vehicle at scenario start, in metres.
     * Positive = ahead. Use negative for objects behind the vehicle.
     */
    private double distanceMetres;

    /**
     * Lateral offset at scenario start, in metres.
     * Positive = right of vehicle centreline, negative = left.
     * Used to derive initial bearingDegrees based on distance.
     */
    private double lateralOffsetMetres;

    /** Object's own absolute speed in km/h (0.0 for stationary objects) */
    private double objectSpeedKmh;

    /**
     * Heading of this object in degrees.
     * 0 = travelling in same direction as ego, 180 = head-on.
     * 90 = crossing from left, -90 = crossing from right.
     */
    private double headingDegrees;

    /**
     * Whether this object is visible to sensors.
     * Can be toggled by a ScenarioEvent (e.g. pedestrian steps out suddenly).
     */
    private boolean visible;

    /**
     * Time in seconds after scenario start when this object becomes visible.
     * 0.0 = visible from the start.
     */
    public final double appearsAtSec;

    /**
     * Time in seconds after scenario start when this object disappears.
     * Double.MAX_VALUE = present for the whole scenario.
     */
    public final double disappearsAtSec;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Creates a world object visible from the start of the scenario.
     *
     * @param id                   unique identifier
     * @param objectClass          classification for camera
     * @param distanceMetres       initial distance ahead of ego vehicle
     * @param lateralOffsetMetres  initial lateral offset (+ = right)
     * @param objectSpeedKmh       object's own absolute speed
     */
    public WorldObject(final String id,
                       final ObjectClass objectClass,
                       final double distanceMetres,
                       final double lateralOffsetMetres,
                       final double objectSpeedKmh) {
        this(id, objectClass, distanceMetres, lateralOffsetMetres,
             objectSpeedKmh, 0.0, 0.0, Double.MAX_VALUE);
    }

    /**
     * Full constructor with heading, appearance, and disappearance times.
     *
     * @param id                   unique identifier
     * @param objectClass          classification for camera
     * @param distanceMetres       initial distance ahead
     * @param lateralOffsetMetres  initial lateral offset (+ = right)
     * @param objectSpeedKmh       object's own absolute speed
     * @param headingDegrees       direction of travel (0 = same as ego)
     * @param appearsAtSec         time in seconds when object becomes visible
     * @param disappearsAtSec      time in seconds when object leaves scene
     */
    public WorldObject(final String id,
                       final ObjectClass objectClass,
                       final double distanceMetres,
                       final double lateralOffsetMetres,
                       final double objectSpeedKmh,
                       final double headingDegrees,
                       final double appearsAtSec,
                       final double disappearsAtSec) {
        this.id                  = id;
        this.objectClass         = objectClass;
        this.distanceMetres      = distanceMetres;
        this.lateralOffsetMetres = lateralOffsetMetres;
        this.objectSpeedKmh      = objectSpeedKmh;
        this.headingDegrees      = headingDegrees;
        this.appearsAtSec        = appearsAtSec;
        this.disappearsAtSec     = disappearsAtSec;
        this.visible             = (appearsAtSec == 0.0);
    }

    // -------------------------------------------------------------------------
    // Physics update
    // -------------------------------------------------------------------------

    /**
     * Updates the object's position given a time step.
     *
     * The ego vehicle's motion is handled by the generator — this method
     * only advances the object's own position based on its heading and speed.
     * Ego vehicle motion is factored in by the generator calling this AND
     * then adjusting distanceMetres and lateralOffsetMetres.
     *
     * @param dtSeconds  time step in seconds
     */
    public void advanceOwnMotion(final double dtSeconds) {
        final double headingRad = Math.toRadians(headingDegrees);
        final double distPerStep = (objectSpeedKmh / 3.6) * dtSeconds;

        // Forward component reduces distance-ahead (objects travelling same
        // direction as ego move forward; we adjust relative to ego separately)
        distanceMetres      += distPerStep * Math.cos(headingRad);
        lateralOffsetMetres += distPerStep * Math.sin(headingRad);
    }

    /**
     * Adjusts position to account for ego vehicle moving forward.
     * Called after advanceOwnMotion() each tick.
     *
     * @param egoDistanceAdvancedMetres  metres the ego vehicle moved forward this step
     */
    public void adjustForEgoMotion(final double egoDistanceAdvancedMetres) {
        distanceMetres -= egoDistanceAdvancedMetres;
    }

    /**
     * Rotates the object's bearing as the ego vehicle turns.
     * A right turn shifts all object bearings to the left (negative direction).
     *
     * @param turnDeltaDegrees  positive = ego turning right, negative = left
     */
    public void adjustForEgoTurn(final double turnDeltaDegrees) {
        lateralOffsetMetres = getBearingDegrees() - turnDeltaDegrees;
        // Recompute lateral from the new bearing and current distance
        // We store bearing implicitly via (distance, lateralOffset).
        // Rotate the lateral offset by projecting around the new heading.
        final double currentBearing = getBearingDegrees();
        final double newBearing     = currentBearing - turnDeltaDegrees;
        final double dist           = Math.sqrt(distanceMetres * distanceMetres
                                               + lateralOffsetMetres * lateralOffsetMetres);
        final double bearingRad     = Math.toRadians(newBearing);
        this.distanceMetres      = dist * Math.cos(bearingRad);
        this.lateralOffsetMetres = dist * Math.sin(bearingRad);
    }

    // -------------------------------------------------------------------------
    // Visibility
    // -------------------------------------------------------------------------

    public void setVisible(final boolean visible) {
        this.visible = visible;
    }

    public boolean isVisible(final double currentTimeSec) {
        return currentTimeSec >= appearsAtSec && currentTimeSec < disappearsAtSec;
    }

    // -------------------------------------------------------------------------
    // Derived sensor values
    // -------------------------------------------------------------------------

    /**
     * Bearing to this object from the ego vehicle, in degrees.
     * 0 = directly ahead, + = right, - = left.
     * Uses small-angle approximation: atan2(lateral, forward).
     */
    public double getBearingDegrees() {
        if (distanceMetres == 0 && lateralOffsetMetres == 0) {
            return 0.0;
        }
        return Math.toDegrees(Math.atan2(lateralOffsetMetres, distanceMetres));
    }

    /**
     * Straight-line distance to this object from the ego vehicle, in metres.
     */
    public double getDistanceMetres() {
        return Math.sqrt(distanceMetres * distanceMetres
                         + lateralOffsetMetres * lateralOffsetMetres);
    }

    /**
     * Relative speed of this object with respect to the ego vehicle, in km/h.
     *
     * Negative = object is approaching (closing distance).
     * Positive = object is moving away.
     *
     * @param egoSpeedKmh  current ego vehicle speed
     */
    public double getRelativeSpeedKmh(final double egoSpeedKmh) {
        // Component of object velocity along the line of sight to ego
        final double headingRad = Math.toRadians(headingDegrees);
        final double objectForwardKmh = objectSpeedKmh * Math.cos(headingRad);
        // Relative speed: positive object forward component means it moves
        // in same direction as ego, reducing closure rate.
        return objectForwardKmh - egoSpeedKmh;
    }

    // -------------------------------------------------------------------------
    // Mutators for scenario events
    // -------------------------------------------------------------------------

    public void setObjectSpeedKmh(final double speedKmh) {
        this.objectSpeedKmh = speedKmh;
    }

    public void setHeadingDegrees(final double heading) {
        this.headingDegrees = heading;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    public double getRawDistanceAhead()  { return distanceMetres; }
    public double getRawLateralOffset()  { return lateralOffsetMetres; }
    public double getObjectSpeedKmh()    { return objectSpeedKmh; }
    public double getHeadingDegrees()    { return headingDegrees; }
    public ObjectClass getObjectClass()  { return objectClass; }

    /** Converts ObjectClass to the Camera objectType string used in JSON */
    public String toCameraObjectType() {
        return switch (objectClass) {
            case VEHICLE           -> "VEHICLE";
            case PEDESTRIAN        -> "PEDESTRIAN";
            case CYCLIST           -> "CYCLIST";
            case STATIONARY_OBJECT -> "STATIONARY_OBJECT";
            default                -> "UNKNOWN";
        };
    }
}
