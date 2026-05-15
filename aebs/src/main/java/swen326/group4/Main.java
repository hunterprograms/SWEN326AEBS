package swen326.group4;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        new Camera("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("3", "aebs/src/test/java/Simulator/Scenarios").start();

        new Lidar("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("3", "aebs/src/test/java/Simulator/Scenarios").start();

        new Radar("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Radar("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Radar("3", "aebs/src/test/java/Simulator/Scenarios").start();
    }
}