package swen326.group4;

import java.io.IOException;

import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

public class Main {

    public static void main(final String[] args) {
        try {
            new WheelSensor(1, "aebs/src/main/java/swen326/group4/worldWheelSpeedId1.json").start();
        } catch (IOException e) {
            System.err.println("Failed to open world file: " + e.getMessage());
        }
    }
}