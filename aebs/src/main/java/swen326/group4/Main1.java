package swen326.group4;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

/**
 * Main1 — Live Scenario Run (File Output Version).
 * * Starts all sensors from real scenario JSON files, executes the controller,
 * and writes the entire log output to a scenario_output.txt file.
 */
public class Main1 {

    private static final String DATA_DIRECTORY = "/Users/marstetom/2026 Work/SWEN326/SWEN326AEBS/scenarios/SC-002";
    private static final int RUN_DURATION_MS = 8_000;
    /* Define the target output text file */
    private static final String OUTPUT_FILE_PATH = "scenario_output.txt";

    public static void main(final String[] args) throws InterruptedException {

        // ---------------------------------------------------------------------
        // Redirect System.out to a text file
        // ---------------------------------------------------------------------
        try {
            PrintStream fileOut = new PrintStream(new File(OUTPUT_FILE_PATH));
            // Redirect standard output to the file stream
            System.setOut(fileOut);
            
            // Optional: If you want to log errors to the file as well:
            // System.setErr(fileOut);
            
        } catch (FileNotFoundException e) {
            System.err.println("Error: Could not create or write to output file: " + OUTPUT_FILE_PATH);
            e.printStackTrace();
            return;
        }

        /* All subsequent System.out.println calls will go into the .txt file */
        System.out.println("=== AEBS Main1: Live Scenario Run ===");
        System.out.println("Data directory : " + DATA_DIRECTORY);
        System.out.println("Output File    : " + OUTPUT_FILE_PATH);
        System.out.println("Run duration   : " + RUN_DURATION_MS + "ms");
        System.out.println("======================================");

        // 1. Construct all sensor instances
        final Camera cam1 = new Camera("1", DATA_DIRECTORY);
        final Camera cam2 = new Camera("2", DATA_DIRECTORY);
        final Camera cam3 = new Camera("3", DATA_DIRECTORY);

        final Radar radar1 = new Radar("1", DATA_DIRECTORY);
        final Radar radar2 = new Radar("2", DATA_DIRECTORY);
        final Radar radar3 = new Radar("3", DATA_DIRECTORY);

        final Lidar lidar1 = new Lidar("1", DATA_DIRECTORY);
        final Lidar lidar2 = new Lidar("2", DATA_DIRECTORY);
        final Lidar lidar3 = new Lidar("3", DATA_DIRECTORY);

        final WheelSensor ws1 = new WheelSensor("1", DATA_DIRECTORY);
        final WheelSensor ws2 = new WheelSensor("2", DATA_DIRECTORY);
        final WheelSensor ws3 = new WheelSensor("3", DATA_DIRECTORY);

        final Driver driver = new Driver(DATA_DIRECTORY);

        // 2. Construct stub escalation channels
        final BrakingController.EscalationChannel channelA = new BrakingController.EscalationChannel() {
            private boolean confirmed = false;

            @Override
            public void sendAlert() {
                System.out.println("[ChannelA] *** ESCALATION ALERT *** (audio/visual)");
                confirmed = true;
            }

            @Override
            public boolean isDeliveryConfirmed() { return confirmed; }

            @Override
            public String channelName() { return "ChannelA-AudioVisual"; }
        };

        final BrakingController.EscalationChannel channelB = new BrakingController.EscalationChannel() {
            private boolean confirmed = false;

            @Override
            public void sendAlert() {
                System.out.println("[ChannelB] *** ESCALATION ALERT *** (haptic/dashboard)");
                confirmed = true;
            }

            @Override
            public boolean isDeliveryConfirmed() { return confirmed; }

            @Override
            public String channelName() { return "ChannelB-HapticDashboard"; }
        };

        // 3. Construct the controller
        final BrakingController controller = new BrakingController(
            cam1, cam2, cam3,
            radar1, radar2, radar3,
            lidar1, lidar2, lidar3,
            ws1, ws2, ws3,
            driver,
            channelA, channelB
        );

        // 4. Start all sensors
        System.out.println("\n--- Starting sensors ---");
        cam1.start(); cam2.start(); cam3.start();
        radar1.start(); radar2.start(); radar3.start();
        lidar1.start(); lidar2.start(); lidar3.start();
        ws1.start(); ws2.start(); ws3.start();
        driver.start();

        Thread.sleep(200);

        // 5. Start the controller
        System.out.println("\n--- Starting controller ---");
        controller.start();

        // 6. Run execution loop
        final long startTime = System.currentTimeMillis();
        final long endTime = startTime + RUN_DURATION_MS;

        while (System.currentTimeMillis() < endTime) {
            Thread.sleep(500);
            final long elapsed = System.currentTimeMillis() - startTime;
            System.out.println("\n[t=" + elapsed + "ms] Last decision: " + controller.getLastDecision());
        }

        // 7. Stop all components
        System.out.println("\n--- Stopping controller ---");
        controller.stop();

        System.out.println("--- Stopping sensors ---");
        driver.stop();
        ws3.stop(); ws2.stop(); ws1.stop();
        lidar3.stop(); lidar2.stop(); lidar1.stop();
        radar3.stop(); radar2.stop(); radar1.stop();
        cam3.stop(); cam2.stop(); cam1.stop();

        System.out.println("\n=== Main1 complete ===");
        
        // Explicitly close the custom print stream to flush data safely
        System.out.close(); 
    }
}