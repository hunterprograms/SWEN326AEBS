package swen326.group4.Display;

/**
 * Coordinate structural data pipelines linking the environmental execution loops 
 * to the Driver Information Display Model component.
 * <p>
 * This controller intercepts incoming sensor packets and delegates transactional 
 * state grouping updates directly to the underlying model container, avoiding 
 * localized business logic leaks.
 * </p>
 */
public class DIDController {
    /** The single source of truth model instance managed by this platform subsystem. */
    private final DIDModel model;

    /**
     * Constructs a control pipeline wired directly to an active system display model.
     *
     * @param model the target system data model container (must not be null)
     * @throws AssertionError if the dependent model reference is null
     */
    public DIDController(DIDModel model) {
        assert model != null : "Structural Exception - Controller cannot bind to an uninitialized Model pointer.";
        this.model = model;
    }

    /**
     * Forwards a processed vehicle tracking matrix to update display telemetry fields.
     *
     * @param speed    the raw sensor telemetry wheel speed data entry in km/h
     * @param distance the computed target distance estimation in meters
     * @param ttc      the calculated situational time-to-collision rating in seconds
     */
    public void updateVehicleMetrics(double speed, double distance, double ttc) {
        model.updateTelemetry(speed, distance, ttc);
    }

    /**
     * Forwards automated safety engagement flags down to the model visualization layer.
     *
     * @param brakingActive true if autonomous deceleration is active, false otherwise
     * @param alarmActive   true if audio warnings are sounding, false otherwise
     * @param errorMargin   the real-time deviation percentage tracking deceleration success
     */
    public void updateInterventionMetrics(boolean brakingActive, boolean alarmActive, double errorMargin) {
        model.updateInterventions(brakingActive, alarmActive, errorMargin);
    }

    /**
     * Transitions the operational mode state machine configuration of the platform.
     *
     * @param state the target execution mode transition vector (ACTIVE, MAINTENANCE, DISABLED)
     * @throws AssertionError if a null system transition parameter state is supplied
     */
    public void changeSystemState(DIDModel.SystemState state) {
        assert state != null : "State Exception - Target system initialization mode cannot be null.";
        model.setSystemState(state);
    }

    public void updateSpeed(double speed) {
        model.updateSpeed(speed);
    }

    public void updateDistance(double distance) {
        model.updateDistance(distance);
    }

    public void updateTTC(double ttc) { // flaot to double
        model.updateTTC(ttc);
    }

    public void updateBrakingActive(boolean brakingActive) {
        model.updateBrakingActive(brakingActive);
    }

    public void updateAlarmActive(boolean alarmActive) {
        model.updateAlarmActive(alarmActive);
    }

    public void updateErrorMargin(double errorMargin) {
        model.updateErrorMargin(errorMargin);
    }

    /** Surfaces a residual collision risk alert on the DID (FR-3110). */
    public void setResidualCollisionAlert(boolean residualCollisionAlert) {
        model.setResidualCollisionAlert(residualCollisionAlert);
    }

    /** Surfaces a sensor fault warning on the DID and forces MAINTENANCE state (FR-2102). */
    public void showSensorFaultWarning() {
        model.setSensorFaultAlert(true); // state change handled inside model
    }

    /** Surfaces a wheel lockup alert on the DID (FR-3112). */
    public void showLockupAlert() {
        model.setLockupAlert(true);
    }

    /** Surfaces a directional instability alert on the DID (FR-3113). */
    public void showDirectionalInstabilityAlert() {
        model.setDirectionalInstabilityAlert(true);
    }

}