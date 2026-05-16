package swen326.group4.Sensors.Radar_Lidar;

/**
 * Common interface for Radar and LiDAR sensor implementations.
 * Each sensor runs on its own clock at 100ms intervals per spec.
 *
 * Implementors are responsible for:
 *   - Reading from their respective world data JSON file on each clock tick
 *   - Exposing the latest reading for consumption by the 2oo3 voter
 *   - Reporting their own status (OK, DEGRADED, FAILED)
 */
public interface RadarLidarSensor {

    enum SensorStatus { OK, DEGRADED, FAILED }

    /**
     * Returns the most recent reading produced by this sensor.
     * Called by the 2oo3 voter after each clock tick.
     * Returns null if no valid reading has been produced yet.
     */
    RadarLidarReading getLatestReading();

    /**
     * Returns the current operational status of this sensor.
     * DEGRADED indicates reduced confidence (e.g. weather).
     * FAILED indicates the sensor should be excluded from voting.
     */
    SensorStatus getStatus();

    /**
     * Starts the internal 100ms clock.
     * Each tick triggers a read from the world data file.
     */
    void start();

    /**
     * Stops the internal clock. Used for clean shutdown in tests.
     */
    void stop();
}