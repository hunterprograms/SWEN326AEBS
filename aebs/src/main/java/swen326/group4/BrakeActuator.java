package swen326.group4;

public class BrakeActuator {
    private static final double MAX_EMERGENCY_DECEL = 30.0; // m/s^2
    private static final double WHEEL_RADIUS_METRES = 0.33; // ~13-inch tire radius
    
    private float currentIntensity = 0.0f;

    public void applyBrake(float intensity) {
        this.currentIntensity = Math.clamp(intensity, 0.0f, 1.0f);
    }

    public float getCurrentIntensity() {
        return this.currentIntensity;
    }

    /**
     * Calculates the wheel RPM drop over a discrete time slice (dt) based on 
     * the active brake pad pressure.
     * * Physics derivation:
     * linear_deceleration = intensity * MAX_EMERGENCY_DECEL
     * angular_deceleration = linear_deceleration / tire_radius
     * rpm_drop = angular_deceleration * (60 / 2pi) * dt
     */
    public double calculateDeceleratedRpm(double currentRpm, double timeDeltaSeconds) {
        if (currentRpm <= 0.0 || currentIntensity <= 0.0f) {
            return Math.max(0.0, currentRpm);
        }

        double linearDecel = this.currentIntensity * MAX_EMERGENCY_DECEL;
        double angularDecel = linearDecel / WHEEL_RADIUS_METRES;
        double rpmDropPerSecond = angularDecel * (60.0 / (2.0 * Math.PI));

        return Math.max(0.0, currentRpm - (rpmDropPerSecond * timeDeltaSeconds));
    }
}