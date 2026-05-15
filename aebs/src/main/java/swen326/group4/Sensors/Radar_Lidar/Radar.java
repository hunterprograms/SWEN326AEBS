package swen326.group4.Sensors.Radar_Lidar;

import java.util.ArrayList;

public class Radar{
    private enum SensorStatus{
        OK,
        DEGRADED,
        FAILED,
    }
    private SensorStatus status; 
    private ArrayList<RadarLidarReading> previousReading;

    private static final int UPDATE_INTERVAL_MS = 100;

    // Called by simulator every 100ms
    public void update(ArrayList<RadarLidarReading> reading) {
        previousReading = reading;
      }

    // Called by 2oo3 voter
    public ArrayList<RadarLidarReading> getLatestReading() {  
        return previousReading;
    }

    public SensorStatus getStatus() { return status; }
    public int getFrequency() { return UPDATE_INTERVAL_MS; }
}
