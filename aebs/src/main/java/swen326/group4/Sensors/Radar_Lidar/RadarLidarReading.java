package swen326.group4.Sensors.Radar_Lidar;

import java.util.List;

/**
 * Immutable data container for a single Radar/LiDAR sensor reading.
 * Derived from world state data in worldLidarData.json / worldRadarData.json.
 *
 * distanceMetres   : distance to nearest detected obstacle (0.5 - 200m per spec)
 * relativeSpeedKmh : speed of obstacle relative to vehicle, negative = approaching
 * bearingDegrees   : angle to obstacle relative to vehicle heading, 0 = directly ahead
 * confidenceScore  : 0.0 (no confidence) to 1.0 (full confidence), degraded by weather/damage
 * timestampMs      : system time at point of reading
 */
public record RadarLidarReading(
    List<DetectedObject> detectedObjects,
    float confidenceScore,
    long  timestampMs
) {}