package swen326.group4.Display;

/**
 * Represent the internal state of the AEBS system.
 */
public class DIDModel {
    
    public enum SystemState { ACTIVE, MAINTENANCE, DISABLED }

    // 4.1: Controls for setting AEBS sensitivity
    private double sensitivityThreshold = 1.5; // Default intervention threshold
    private static final double MIN_THRESHOLD = 0.5;
    private static final double MAX_THRESHOLD = 5.0;

    // 4.4 Sensor Data Ranges
    private SystemState systemState = SystemState.ACTIVE;
    private double currentSpeed = 0.0;          // 0 t0 250 km/h
    private double distanceToHazard = 200.0;    // 0.5 to 200 meters
    private double timeToCollision = 99.9;
    private boolean brakingActive = false;

    private final AEBSListener[] listeners = new AEBSListener[2];
    private int listenerCount = 0;

    // --- 4.1: Sensitivity Controls ---
    public void setSensitivityThreshold(double value) {
        // Design by Contract: Ensure sensitivity is within specified operational limits
        assert value >= MIN_THRESHOLD && value <= MAX_THRESHOLD : "Invalid Sensitivity: " + value;
        this.sensitivityThreshold = value;
        notifyListeners();
    }  

    // --- 4.4: Setters with Specification Assertions ---
    public void setCurrentSpeed(double value) {
        // Spec 4.4: Wheel Speed Data 0 to 250 km/h
        assert value >= 0.0 && value <= 250.0 : "Speed Out of Range: " + value;
        this.currentSpeed = value;
        notifyListeners();
    }

    public void setDistanceToHazard(double value) {
        // Spec 4.4: Radar/Lidar detection from 0.5 to 200 meters
        assert value >= 0.0 && value <= 200.0 : "Distance Out of Range: " + value;
        this.distanceToHazard = value;
        notifyListeners();
    }

    public void setSystemState(SystemState newState) {
        if (newState == SystemState.DISABLED) {
            // Safety Check: Manual deactivation (4.1) only safe when stationary
            assert currentSpeed == 0 : "HAZARD: Manual deactivation attempted while in motion!";
        }
        this.systemState = newState;
        notifyListeners();
    }

    public void setTimeToCollision(double value) {
        assert value >= 0.0 : "Logic Error: Negative TTC";
        this.timeToCollision = value;
        notifyListeners();
    }

    public void setBrakingActive(boolean value) {
        if (value) {
            // 4.1: Feedback for maintenance needs
            assert systemState != SystemState.MAINTENANCE : "Hardware Failure: Braking prohibited";
        }
        this.brakingActive = value;
        notifyListeners();
    }

    /*
     * --- Getters ---
     */
    public SystemState getSystemState() { return systemState; }
    public double getCurrentSpeed() { return currentSpeed; }
    public double getDistanceToHazard() { return distanceToHazard; }
    public double getTimeToCollision() { return timeToCollision; }
    public double getSensitivityThreshold() { return sensitivityThreshold; }
    public boolean isBrakingActive() { return brakingActive; }

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
            listeners[i].stateChanged(this);
        }
    }

}
