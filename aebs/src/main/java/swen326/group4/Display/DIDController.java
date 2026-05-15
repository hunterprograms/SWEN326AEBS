package swen326.group4.Display;

import javax.swing.Timer;

/**
 * The System Controller (Domain Logic).
 * Adheres to safety-critical standards by separating logic from the UI.
 */
public class DIDController {
    private final DIDModel model;
    private final Timer controlLoop;
    
    // Constant timing and physics for predictability (Rule 2)
    private static final int TICK_RATE_MS = 50; 
    private static final double TICK_IN_SECONDS = TICK_RATE_MS / 1000.0;
    private static final double DECELERATION_STRENGTH = 0.8; 

    public DIDController(DIDModel model) {
        this.model = model;
        
        // Fixed-rate heartbeat prevents unbounded execution (Rule 2)
        this.controlLoop = new Timer(TICK_RATE_MS, e -> runSafetyCycle());
    }

    public void start() {
        controlLoop.start();
    }

    /**
     * The primary safety cycle. 
     * Kept under 60 lines to satisfy Rule 6.
     */
    private void runSafetyCycle() {
        double speed = model.getCurrentSpeed();
        double distance = model.getDistanceToHazard();
        double threshold = model.getSensitivityThreshold();

        // Rule 5: Assert that inputs are within physical possibility
        assert speed >= 0 : "Speed cannot be negative";
        assert distance >= 0 : "Distance cannot be negative";

        // 1. Logic: Calculate Time to Collision
        double ttc = calculateTTC(speed, distance);

        // 2. Decision: Threshold comparison
        if (speed > 0 && ttc <= threshold) {
            applyBrakes(speed);
        } else {
            monitorReset(speed);
        }

        // 3. Physics: Advance the vehicle state
        updatePosition(speed, distance);
    }

    private double calculateTTC(double speed, double distance) {
        // Guard against division by zero (Rule 5/9 logic)
        if (speed <= 0.01) {
            return Double.MAX_VALUE;
        }
        return distance / speed;
    }

    private void applyBrakes(double currentSpeed) {
        model.setBrakingActive(true);
        // Gradually reduce speed to 0
        double nextSpeed = Math.max(0, currentSpeed - DECELERATION_STRENGTH);
        model.setCurrentSpeed(nextSpeed);
    }

    private void monitorReset(double speed) {
        // Only release brakes if the car has successfully stopped
        if (speed <= 0) {
            model.setBrakingActive(false);
        }
    }

    private void updatePosition(double speed, double distance) {
        if (speed > 0 && distance > 0) {
            double deltaDist = speed * TICK_IN_SECONDS;
            model.setDistanceToHazard(Math.max(0, distance - deltaDist));
        }
    }
}
