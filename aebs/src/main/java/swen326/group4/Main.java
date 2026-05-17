package swen326.group4;

<<<<<<< HEAD
import swen326.group4.Audio.AuditoryController;
import swen326.group4.Display.*;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

public class Main {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            // 1. Setup the core components
            DIDModel model = new DIDModel();
            DIDView view = new DIDView(model);
            DIDController controller = new DIDController(model);

            // 2. Connect the audio listener
            AuditoryController audio = new AuditoryController();
            model.addListener(audio);

            // 3. Setup the UI Window
            JFrame frame = new JFrame("AEBS Monitoring System - NORMAL");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(view);
            frame.pack();
            frame.setVisible(true);

            /* * NORMAL STATE:
             * Braking: false
             * Alarm:   false
             * Error:   0.0
             * 
             * WARNING STATE:
             * Braking: false
             * Alarm: true
             * Error: 1.2
             * 
             * BRAKING STATE:
             * Braking: true
             * Alarm: true
             * Error: 0.0
             */
            controller.updateInterventionMetrics( false, false, 0.0);
            
            System.out.println("AEBS initialized: System is in NORMAL monitoring mode.");
        });
=======
import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

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

        new Driver("aebs/src/test/java/Simulator/Scenarios").start();

        new WheelSensor("2", "aebs/src/test/java/Simulator/Scenarios").start();
>>>>>>> origin/Integration
    }
}