package swen326.group4.Sensors.Driver;

/**
 * Immutable data record produced by Driver on each 250ms tick.
 *
 * Passed to AEBS Control alongside sensor readings to determine
 * whether the driver is already responding to a hazard.
 *
 * Requirement traceability:
 *   HF-001 : Carries driver action used to detect false brake response
 *   HF-002 : Carries NONE action used to detect inattentive driver
 */
public final class DriverReading {

    /** Driver action for this tick */
    private final Driver.Action action;

    /** Timestamp of this tick in milliseconds */
    private final long timestampMs;

    /**
     * Constructs a DriverReading.
     * @param action      BRAKE or NONE
     * @param timestampMs tick timestamp
     */
    public DriverReading(final Driver.Action action, final long timestampMs) {
        assert action != null : "action must not be null";
        assert timestampMs >= 0 : "timestampMs must not be negative";
        this.action      = action;
        this.timestampMs = timestampMs;
    }

    public Driver.Action action()  { return action; }
    public long timestampMs()      { return timestampMs; }

    /** Returns true if the driver is braking this tick. */
    public boolean isBraking() {
        assert action != null : "action must not be null";
        return action == Driver.Action.BRAKE;
    }
}