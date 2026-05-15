package swen326.group4.Sensors.Radar_Lidar;

public record DetectedObject(
    float distanceMetres,
    float relativeSpeedKmh,
    float bearingDegrees
) {}