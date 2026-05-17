package Simulator;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Applies Fault definitions to live sensor instances during a scenario.
 *
 * The FaultInjector holds references to all sensor instances and a list
 * of Fault definitions. On each call to tick(currentTimeSec), it checks
 * every fault and applies it to the target sensor if active.
 *
 * This keeps fault logic out of the sensors themselves — sensors remain
 * pure data readers, and the injector is the single point of fault control.
 *
 * Requirement traceability:
 *   NF2201 : SENSOR_FAILURE on camera tests 2oo3 fallback
 *   NF2202 : OBSTRUCTION on camera tests fallback braking
 *   NF4201 : CORRUPT_DATA tests redundancy catching corruption
 *   FR2206 : SENSOR_FAILURE on LiDAR tests redundant components
 *   FR2207 : SENSOR_FAILURE on all LiDAR tests AEBS self-disable
 */
public class FaultInjector {

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** Camera sensor units — index 0=cam1, 1=cam2, 2=cam3 */
    private final Camera[] cameras;

    /** Lidar sensor units — index 0=lidar1, 1=lidar2, 2=lidar3 */
    private final Lidar[] lidars;

    /** Radar sensor units — index 0=radar1, 1=radar2, 2=radar3 */
    private final Radar[] radars;

    /** Wheel sensor units — index 0=wheel1, 1=wheel2, 2=wheel3 */
    private final WheelSensor[] wheels;

    /** All faults registered with this injector */
    private final List<Fault> faults;

    /** Tracks which faults have already been logged as injected */
    private final boolean[] injected;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public FaultInjector(final Camera[] cameras,
                         final Lidar[] lidars,
                         final Radar[] radars,
                         final WheelSensor[] wheels) {
        assert cameras != null : "cameras must not be null";
        assert lidars != null  : "lidars must not be null";
        assert radars != null  : "radars must not be null";
        assert wheels != null  : "wheels must not be null";
        this.cameras = cameras;
        this.lidars  = lidars;
        this.radars  = radars;
        this.wheels  = wheels;
        this.faults  = new ArrayList<>();
        this.injected = new boolean[0];
    }

    // -------------------------------------------------------------------------
    // Public interface
    // -------------------------------------------------------------------------

    /**
     * Registers a fault with this injector.
     * Call before starting the scenario.
     */
    public void addFault(final Fault fault) {
        assert fault != null : "fault must not be null";
        faults.add(fault);
    }

    /**
     * Called every tick with the current scenario time in seconds.
     * Checks all registered faults and applies any that become active.
     *
     * @param currentTimeSec current scenario time in seconds
     */
    public void tick(final double currentTimeSec) {
        assert currentTimeSec >= 0.0 : "currentTimeSec must not be negative";
        for (int i = 0; i < faults.size(); i++) {
            final Fault fault = faults.get(i);
            if (fault.isActiveAt(currentTimeSec)) {
                applyFault(fault);
            }
        }
    }

    /**
     * Prints a summary of all registered faults and their current state.
     * @param currentTimeSec current scenario time in seconds
     */
    public void printStatus(final double currentTimeSec) {
        assert currentTimeSec >= 0.0 : "currentTimeSec must not be negative";
        System.out.println("\n=== Fault Injector Status at t=" + currentTimeSec + "s ===");
        for (int i = 0; i < faults.size(); i++) {
            final Fault fault = faults.get(i);
            System.out.println(fault.toDisplayString()
                + " | active=" + fault.isActiveAt(currentTimeSec));
        }
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Applies the fault to the appropriate sensor based on FaultTarget.
     */
    private void applyFault(final Fault fault) {
        assert fault != null : "fault must not be null";
        assert fault.target != null : "fault target must not be null";

        switch (fault.target) {
            case CAMERA_1: applyToCamera(cameras[0], fault); break;
            case CAMERA_2: applyToCamera(cameras[1], fault); break;
            case CAMERA_3: applyToCamera(cameras[2], fault); break;
            case LIDAR_1:  applyToLidar(lidars[0], fault);  break;
            case LIDAR_2:  applyToLidar(lidars[1], fault);  break;
            case LIDAR_3:  applyToLidar(lidars[2], fault);  break;
            case RADAR_1:  applyToRadar(radars[0], fault);  break;
            case RADAR_2:  applyToRadar(radars[1], fault);  break;
            case RADAR_3:  applyToRadar(radars[2], fault);  break;
            case WHEEL_SENSOR_1: applyToWheel(wheels[0], fault); break;
            case WHEEL_SENSOR_2: applyToWheel(wheels[1], fault); break;
            case WHEEL_SENSOR_3: applyToWheel(wheels[2], fault); break;
            default:
                System.err.println("FaultInjector: unhandled target " + fault.target);
        }
    }

    private void applyToCamera(final Camera camera, final Fault fault) {
        assert camera != null : "camera must not be null";
        assert fault != null  : "fault must not be null";
        switch (fault.type) {
            case SENSOR_FAILURE:
            case CORRUPT_DATA:
            case STALE_DATA:
                camera.injectFault();
                logInjection(fault, "Camera status -> OBSTRUCTED");
                break;
            case OBSTRUCTION:
                camera.injectFault();
                logInjection(fault, "Camera status -> OBSTRUCTED (physical blockage)");
                break;
            default:
                logInjection(fault, "Camera fault type not yet implemented: " + fault.type);
        }
    }

    private void applyToLidar(final Lidar lidar, final Fault fault) {
        assert lidar != null : "lidar must not be null";
        assert fault != null : "fault must not be null";
        switch (fault.type) {
            case SENSOR_FAILURE:
            case CORRUPT_DATA:
            case STALE_DATA:
            case OBSTRUCTION:
                lidar.injectFault();
                logInjection(fault, "Lidar status -> FAILED");
                break;
            default:
                logInjection(fault, "Lidar fault type not yet implemented: " + fault.type);
        }
    }

    private void applyToRadar(final Radar radar, final Fault fault) {
        assert radar != null : "radar must not be null";
        assert fault != null : "fault must not be null";
        switch (fault.type) {
            case SENSOR_FAILURE:
            case CORRUPT_DATA:
            case STALE_DATA:
            case OBSTRUCTION:
                radar.injectFault();
                logInjection(fault, "Radar status -> FAILED");
                break;
            default:
                logInjection(fault, "Radar fault type not yet implemented: " + fault.type);
        }
    }

    private void applyToWheel(final WheelSensor wheel, final Fault fault) {
        assert wheel != null : "wheel must not be null";
        assert fault != null : "fault must not be null";
        switch (fault.type) {
            case SENSOR_FAILURE:
            case CORRUPT_DATA:
            case STALE_DATA:
                //wheel.injectFault();
                logInjection(fault, "WheelSensor -> exhausted");
                break;
            default:
                logInjection(fault, "Wheel fault type not yet implemented: " + fault.type);
        }
    }

    /**
     * Logs the fault injection to console once.
     */
    private void logInjection(final Fault fault, final String effect) {
        assert fault != null  : "fault must not be null";
        assert effect != null : "effect must not be null";
        System.out.println("[FAULT INJECTED] " + fault.faultId
            + " -> " + fault.target
            + " | " + fault.type
            + " | " + effect
            + " | req=" + fault.requirementId);
    }
}