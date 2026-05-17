package Simulator;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Camera.CameraVoter;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

public class Tests {

    private static final String SCENARIOS = "aebs/src/test/java/Simulator/Scenarios";

    public static void main(String[] args) throws InterruptedException {

        System.out.println("=== AEBS Fault Injection Test ===\n");

        // --- Sensors ---
        final Camera cam1 = new Camera("1", SCENARIOS);
        final Camera cam2 = new Camera("2", SCENARIOS);
        final Camera cam3 = new Camera("3", SCENARIOS);

        final Lidar lidar1 = new Lidar("1", SCENARIOS);
        final Lidar lidar2 = new Lidar("2", SCENARIOS);
        final Lidar lidar3 = new Lidar("3", SCENARIOS);

        final Radar radar1 = new Radar("1", SCENARIOS);
        final Radar radar2 = new Radar("2", SCENARIOS);
        final Radar radar3 = new Radar("3", SCENARIOS);

        final WheelSensor wheel1 = new WheelSensor("1", SCENARIOS);
        final WheelSensor wheel2 = new WheelSensor("2", SCENARIOS);
        final WheelSensor wheel3 = new WheelSensor("3", SCENARIOS);

        // --- Camera voter ---
        final CameraVoter voter = new CameraVoter(cam1, cam2, cam3);

        // --- Fault definitions ---
        final Fault cameraFault = new Fault.Builder(
                "F-001", Fault.FaultTarget.CAMERA_1,
                Fault.FaultType.SENSOR_FAILURE, 1.0)
            .description("Camera 1 hardware failure mid-scenario")
            .durationSec(0.0)
            .requirementId("NF2201")
            .build();

        final Fault lidarFault = new Fault.Builder(
                "F-002", Fault.FaultTarget.LIDAR_2,
                Fault.FaultType.OBSTRUCTION, 1.5)
            .description("Lidar 2 weather obstruction")
            .durationSec(0.0)
            .requirementId("FR2206")
            .build();

        final Fault radarFault = new Fault.Builder(
                "F-003", Fault.FaultTarget.RADAR_3,
                Fault.FaultType.CORRUPT_DATA, 2.0)
            .description("Radar 3 corrupt data")
            .durationSec(0.0)
            .requirementId("NF4201")
            .build();

        final Fault wheelFault = new Fault.Builder(
                "F-004", Fault.FaultTarget.WHEEL_SENSOR_1,
                Fault.FaultType.STALE_DATA, 2.5)
            .description("Wheel sensor 1 stale data dropout")
            .durationSec(0.0)
            .requirementId("NF3102")
            .build();

        // --- Fault injector ---
        final FaultInjector injector = new FaultInjector(
            new Camera[]{ cam1, cam2, cam3 },
            new Lidar[]{ lidar1, lidar2, lidar3 },
            new Radar[]{ radar1, radar2, radar3 },
            new WheelSensor[]{ wheel1, wheel2, wheel3 }
        );
        injector.addFault(cameraFault);
        injector.addFault(lidarFault);
        injector.addFault(radarFault);
        injector.addFault(wheelFault);

        // --- Start all sensors ---
        System.out.println("--- Starting Sensors ---");
        cam1.start();
        cam2.start();
        cam3.start();
        lidar1.start();
        lidar2.start();
        lidar3.start();
        radar1.start();
        radar2.start();
        radar3.start();
        wheel1.start();
        wheel2.start();
        wheel3.start();

        // --- Before injection snapshot ---
        Thread.sleep(500);
        System.out.println("\n--- Sensor Status BEFORE Fault Injection ---");
        printCameraStatus(cam1, cam2, cam3, voter);
        System.out.println("Lidar  1: " + lidar1.getStatus()
            + " | Lidar  2: " + lidar2.getStatus()
            + " | Lidar  3: " + lidar3.getStatus());
        System.out.println("Radar  1: " + radar1.getStatus()
            + " | Radar  2: " + radar2.getStatus()
            + " | Radar  3: " + radar3.getStatus());
        System.out.println("Wheel  1 exhausted: " + wheel1.isExhausted()
            + " | Wheel 2: " + wheel2.isExhausted()
            + " | Wheel 3: " + wheel3.isExhausted());

        // --- Run scenario with fault injection ---
        System.out.println("\n--- Running Scenario (3 seconds) ---");
        for (int tickMs = 0; tickMs <= 3000; tickMs += 250) {
            Thread.sleep(250);
            final double tickSec = tickMs / 1000.0;
            injector.tick(tickSec);
        }

        // --- After injection snapshot ---
        System.out.println("\n--- Sensor Status AFTER Fault Injection ---");
        printCameraStatus(cam1, cam2, cam3, voter);
        System.out.println("Lidar  1: " + lidar1.getStatus()
            + " | Lidar  2: " + lidar2.getStatus()
            + " | Lidar  3: " + lidar3.getStatus());
        System.out.println("Radar  1: " + radar1.getStatus()
            + " | Radar  2: " + radar2.getStatus()
            + " | Radar  3: " + radar3.getStatus());
        System.out.println("Wheel  1 exhausted: " + wheel1.isExhausted()
            + " | Wheel 2: " + wheel2.isExhausted()
            + " | Wheel 3: " + wheel3.isExhausted());

        // --- Fault report ---
        injector.printStatus(3.0);

        // --- Stop all sensors ---
        cam1.stop(); cam2.stop(); cam3.stop();
        lidar1.stop(); lidar2.stop(); lidar3.stop();
        radar1.stop(); radar2.stop(); radar3.stop();
        wheel1.stop(); wheel2.stop(); wheel3.stop();

        System.out.println("\n=== Test Complete ===");
    }

    private static void printCameraStatus(final Camera cam1,
                                          final Camera cam2,
                                          final Camera cam3,
                                          final CameraVoter voter) {
        voter.vote();
        System.out.println("Camera 1: " + cam1.getStatus()
            + " | Camera 2: " + cam2.getStatus()
            + " | Camera 3: " + cam3.getStatus());
        System.out.println("Camera voter: " + voter.getLastVoteResult()
            + " | consensus: " + voter.getConsensusObject()
            + " | eligible: " + voter.getEligibleCount()
            + " | fallback: " + voter.isFallbackActive());
    }
}