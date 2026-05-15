package swen326.group4.Display;

/**
 * Represent the internal state of the AEBS system.
 */
public class DIDModel {
    // System Settings (Mutable Via Controller)
    private double sensitivityThreshold;
    private boolean aebsEnabled;

    // System Status (Updated from Sensor Data)
    private double currentSpeed;
    private double distanceToHazard;
    private double timeToCollision;
    private boolean BrakingActive;

    // Health Status
    private boolean aebsmaintenanceRequired;

    // Constant Safety Limits (Power of Ten)
    private static final double MIN_THRESHOLD = 0.5; // seconds
    private static final double MAX_THRESHOLD = 5.0; // seconds

    // Model Listeners
    private final AEBSListener[] listeners = new AEBSListener[2];
    private int listenerCount = 0;

    /**
     * Get Sensitivity Threshold.
     * @return the sensitivity threshold.
     */
    public double getSensitivityThreshold(){
        return this.sensitivityThreshold;
    }

    /**
     * Is the AEBS enabled.
     * @return the aebs state.
     */
    public boolean isAEBSEnabled(){
        return this.aebsEnabled;
    }

    /**
     * Get Current Speed.
     * @return the current speed.
     */
    public double getCurrentSpeed(){
        return this.currentSpeed;
    }

    /**
     * Get Distance to Hazard.
     * @return the distance to hazard.
     */
    public double getDistanceToHazard(){
        return this.distanceToHazard;
    }

    /**
     * Get Time To Collision.
     * @return the time to collision.
     */
    public double getTimeToCollision(){
        return this.timeToCollision;
    }

    /**
     * Is Braking Active.
     * @return the braking state.
     */
    public boolean isBrakingActive(){
        return this.BrakingActive;
    }

    /**
     * Is AEBS Maintainence Required.
     * @return the AEBS maintainence state.
     */
    public boolean isAEBSMaintanenceRequired(){
        return this.aebsmaintenanceRequired;
    }

    /**
     * Set the sensitivity Threshold.
     */
    public void setSensitivityThreshold(double value){
        assert value >= MIN_THRESHOLD && value <= MAX_THRESHOLD : "Sensitivity out of bounds: " + value;

        if (value < MIN_THRESHOLD) value = MIN_THRESHOLD;
        if (value > MAX_THRESHOLD) value = MAX_THRESHOLD;

        this.sensitivityThreshold = value;
        notifyListeners();
    }

    /**
     * Set the AEBS state.
     */
    public void setAEBSEnabled(boolean value){
        if(value == false) {
            assert currentSpeed == 0 : "HAZARD: Attempted to disable AEBS while in motion!";
        }

        this.aebsEnabled = value;
        notifyListeners();
    }

    /**
     * Set the current speed.
     */
    public void setCurrentSpeed(double value){
        assert value >= 0.0 && value <= 150.0 : "Unrealisitc speed detected: " + value;

        this.currentSpeed = value;
        notifyListeners();
    }

    /**
     * Set the distance to hazard.
     */
    public void setDistanceToHazard(double value) {
        this.distanceToHazard = (value < 0) ? 0 : value;
        notifyListeners();
    }

    /**
     * Set the braking state.
     */
    public void setBrakingActive(boolean value) {
        if (value == true) {
            assert !aebsmaintenanceRequired : "Logic Error: Braking active on failed hardware";
        }

        this.BrakingActive = value;

    }

    /**
     * Set the AEBS maintainence state.
     */
    public void setAEBSMaintanenceRequired(boolean value) {
        if (value == true) {
            assert !this.aebsEnabled : "Warning: Maintanence required while AEBS still enabled";
        }

        this.aebsmaintenanceRequired = value;
        notifyListeners();
    }

    /**
     * Add Listener to the DID view.
     * @param l     the listner.
     */
    public void addListener(AEBSListener l) {
        assert l != null : "Attemped to add a null AEBSListener";

        if (l != null && listenerCount < listeners.length) {
            listeners[listenerCount++] = l;
        }
    }

    /**
     * Notify Listener when model value is updated.
     */
    private void notifyListeners(){
        assert listenerCount >= 0 && listenerCount <= listeners.length : "Invalid lister count: " + listenerCount;

        for (int i = 0; i < listenerCount; i++) {

            assert listeners[i] != null : "Null listener found at index " + i;
            listeners[i].stateChanged();
        }
    }

}
