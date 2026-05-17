package swen326.group4;

import swen326.group4.Car.DIDInterface;
import javax.swing.SwingUtilities;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

public class Main {
    public static void main(String[] args)throws InterruptedException {
        SwingUtilities.invokeLater(() -> {
            DIDInterface systemInterface = new DIDInterface();
            systemInterface.initialize();
        });

        new Camera("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Camera("3", "aebs/src/test/java/Simulator/Scenarios").start();

        new Lidar("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Lidar("3", "aebs/src/test/java/Simulator/Scenarios").start();

        new Radar("1", "aebs/src/test/java/Simulator/Scenarios").start();
        new Radar("2", "aebs/src/test/java/Simulator/Scenarios").start();
        new Radar("3", "aebs/src/test/java/Simulator/Scenarios").start();

        new Driver("aebs/src/test/java/Simulator/Scenarios").start();

        new WheelSensor("2", "aebs/src/test/java/Simulator/Scenarios").start();

    }
}