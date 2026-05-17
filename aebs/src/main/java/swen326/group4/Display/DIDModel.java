package swen326.group4.Display;

/**
 * Represent the internal state of the Driver Information Display (DID) for the AEBS.
 * <p>
 * This class serves as the single source of truth for the display subsystem. It enforces 
 * safety-critical invariants using Design-by-Contract assertions and provides atomic, 
 * transactional update blocks to guarantee data consistency between interrelated metrics.
 * </p>
 * 
 * Implements:
 * <ul>
 *   <li><b>Specification 4.1:</b> Controls for setting AEBS sensitivity thresholds and handling auditory flags.</li>
 *   <li><b>Specification 4.3:</b> Tracking metrics for braking execution success criteria (±5% error margin).</li>
 *   <li><b>Specification 4.4:</b> Strictly enforced data validation ranges for wheel speeds and sensor tracking distances.</li>
 * </ul>
 */
public class DIDModel {
    
    /** Definitive system operational state modes. */
    public enum SystemState { ACTIVE, MAINTENANCE, DISABLED }

    // --- Specification 4.1: Sensitivity Threshold Constants & Fields ---
    private double sensitivityThreshold = 3.0; // Default intervention threshold in seconds
    private static final double MIN_THRESHOLD = 0.5;
    private static final double MAX_THRESHOLD = 5.0;

    // --- Specification 4.4: Core Sensor Data Fields ---
    private SystemState systemState = SystemState.ACTIVE;
    private double currentSpeed = 0.0;        // Validated range: 0.0 to 250.0 km/h
    private double distanceToHazard = 200.0;  // Validated range: 0.5 to 200.0 meters
    private double timeToCollision = 99.9;    // Validated range: Non-negative seconds
    
    // --- Specification 4.1 & 4.3: Intervention & Alert State Flags ---
    private boolean brakingActive = false;
    private boolean alarmActive = false;      // Primary high-priority auditory alert status
    private double brakingErrorMargin = 0.0;  // Tracks deceleration curve accuracy (±5% target)

    // --- Observer Pattern Structures ---
    // Sized to 5 to accommodate multiple view layers, loggers, or audio units safely
    private final AEBSListener[] listeners = new AEBSListener[5];
    private int listenerCount = 0;

    public void updateSpeed(double speed) {
        assert speed >= 0.0 && speed <= 250.0 : "Specification Violation - Speed Out of Range: " + speed;
        this.currentSpeed = speed;
        notifyListeners();
    }

    public void updateDistance(double distance) {
        assert distance >= 0.0 && distance <= 200.0 : "Specification Violation - Distance Out of Range: " + distance;
        this.distanceToHazard = distance;
        notifyListeners();
    }

    public void updateTTC(double ttc) {
        assert ttc >= 0.0 : "Logic Error - Negative Time to Collision value encountered: " + ttc;
        this.timeToCollision = ttc;
        notifyListeners();
    }

    /**
     * Updates primary vehicle telemetry metrics atomically as a single transaction.
     * <p>
     * Enforces design-by-contract constraints upfront to guarantee data consistency. 
     * It ensures that the display thread is notified only once after all interrelated 
     * metrics have been successfully written to the internal state.
     * </p>
     *
     * @param speed    the current longitudinal wheel speed of the vehicle in km/h 
     *                 (must be within operational boundaries [0.0, 250.0])
     * @param distance the direct distance to the tracked forward hazard in meters 
     *                 (must be within sensor detection range [0.0, 200.0])
     * @param ttc      the projected time to collision with the hazard in seconds 
     *                 (must be non-negative)
     * @throws AssertionError if any of the contract invariants are violated
     */
    public void updateTelemetry(double speed, double distance, double ttc) {
        // Enforce specification invariants (Section 4.4)
        assert speed >= 0.0 && speed <= 250.0 : "Specification Violation - Speed Out of Range: " + speed;
        assert distance >= 0.0 && distance <= 200.0 : "Specification Violation - Distance Out of Range: " + distance;
        assert ttc >= 0.0 : "Logic Error - Negative Time to Collision value encountered: " + ttc;

        // Commit transactional state block
        this.currentSpeed = speed;
        this.distanceToHazard = distance;
        this.timeToCollision = ttc;

        // Propagate changes to observers as a single atomic event
        notifyListeners();
    }

    public void updateBrakingActive(boolean brakingActive) {
        if (brakingActive) {
            // Safety critical invariant (Section 4.1 & 4.3)
            assert systemState != SystemState.MAINTENANCE : "Hardware Exception - Braking requested during system fault";
        }
        this.brakingActive = brakingActive;
        notifyListeners();
    }

    public void updateAlarmActive(boolean alarmActive) {
        // assertion ?
        this.alarmActive = alarmActive;
        notifyListeners();
    }

    public void updateErrorMargin(double errorMargin) {
        // assertion ?
        this.brakingErrorMargin = errorMargin;
        notifyListeners();
    }

    /**
     * Updates emergency intervention status flags and performance metrics atomically.
     * <p>
     * This method prevents partial state evaluation across active safety measures. 
     * It guards active braking engagement against critical platform hardware failures 
     * before modifying system parameters.
     * </p>
     *
     * @param brakingActive true if the mechanical automatic braking actuators are engaged, 
     *                      false otherwise
     * @param alarmActive   true if the primary high-priority auditory alert mechanism is firing, 
     *                      false otherwise
     * @param errorMargin   the measured deviation between expected deceleration curves and 
     *                      actual wheel speed drop (evaluated against the ±5% success criteria)
     * @throws AssertionError if automatic braking is requested while the platform is flagged 
     *                        under a maintenance fault state
     */
    public void updateInterventions(boolean brakingActive, boolean alarmActive, double errorMargin) {
        if (brakingActive) {
            // Safety critical invariant (Section 4.1 & 4.3)
            assert systemState != SystemState.MAINTENANCE : "Hardware Exception - Braking requested during system fault";
        }

        // Commit safety state flag metrics
        this.brakingActive = brakingActive;
        this.alarmActive = alarmActive;
        this.brakingErrorMargin = errorMargin;

        // Propagate changes to observers as a single atomic event
        notifyListeners();
    }

    /**
     * Mutates the user-configured sensitivity timeline threshold for AEBS intervention.
     *
     * @param value the desired structural warning horizon gap in seconds
     *              (must strictly be constrained within [0.5, 5.0])
     * @throws AssertionError if the parameter drifts outside specified operational bounds
     */
    public void setSensitivityThreshold(double value) {
        // Design by Contract boundary defense (Section 4.1)
        assert value >= MIN_THRESHOLD && value <= MAX_THRESHOLD : "Invalid Sensitivity Bound: " + value;
        this.sensitivityThreshold = value;
        notifyListeners();
    } 

    /**
     * Transitions the top-level functional mode state machine of the display platform.
     * <p>
     * Includes a fail-safe check preventing manual deactivation sequences unless the 
     * physical vehicle state registers as completely stationary.
     * </p>
     *
     * @param newState the target operational system state (ACTIVE, MAINTENANCE, DISABLED)
     * @throws AssertionError if manual deactivation is attempted while the vehicle is moving
     */
    public void setSystemState(SystemState newState) {
        if (newState == SystemState.DISABLED) {
            // Safety Rule: Manual deactivation (4.1) only safe when stationary
            assert currentSpeed == 0 : "HAZARD - Manual deactivation attempted while vehicle is in motion!";
        }
        this.systemState = newState;
        notifyListeners();
    }

    // --- Safe Inward Getters ---
    public SystemState getSystemState() { return systemState; }
    public double getCurrentSpeed() { return currentSpeed; }
    public double getDistanceToHazard() { return distanceToHazard; }
    public double getTimeToCollision() { return timeToCollision; }
    public double getSensitivityThreshold() { return sensitivityThreshold; }
    public boolean isBrakingActive() { return brakingActive; }
    public boolean isAlarmActive() { return alarmActive; }
    public double getBrakingErrorMargin() { return brakingErrorMargin; }

    /**
     * Attaches an event tracking observer to the display data matrix notification array.
     *
     * @param l the listening component implementation to add (must not be null)
     * @throws AssertionError if an uninitialized, null listener pointer is supplied
     */
    public void addListener(AEBSListener l) {
        assert l != null : "Attempted to bind an uninitialized null AEBSListener system pointer.";
        if (listenerCount < listeners.length) {
            listeners[listenerCount++] = l;
        }
    }

    /**
     * Iterates sequentially through registered system listeners to propagate state updates.
     * Guarantees deterministic upper execution loops loops to align with strict predictability rules.
     */
    private void notifyListeners() {
        // Enforce predictable index boundaries under the Power of Ten loop requirements
        assert listenerCount >= 0 && listenerCount <= listeners.length : "Invalid observer register index count: " + listenerCount;
        for (int i = 0; i < listenerCount; i++) {
            if (listeners[i] != null) {
                listeners[i].stateChanged(this);
            }
        }
    }
}