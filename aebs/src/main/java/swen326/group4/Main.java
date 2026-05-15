package swen326.group4;

import swen326.group4.Sensors.Radar_Lidar.Lidar;

public class Main {
    public static void main(String[] args) {
        new Lidar("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("3", "aebs/src/test/java/Simulator/Scenarios").start();


    }
}