package swen326.group4;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Camera.CameraReading;
import swen326.group4.Sensors.Camera.CameraVoter;

public class Main {
    public static void main(String[] args) throws InterruptedException {
        new Camera("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("3", "aebs/src/test/java/Simulator/Scenarios").start();
    }
}