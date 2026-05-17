package Simulator.ScenarioGenerator;

import java.io.IOException;

/**
 * Entry point for the AEBS scenario file generator.
 *
 * Run with one of:
 *   java SimulatorRunner                     — generates all library scenarios
 *   java SimulatorRunner SC-002              — generates a specific scenario by ID
 *   java SimulatorRunner custom ./output     — generates the custom example scenario
 *
 * All files are written to ./scenarios/<scenarioId>/ by default.
 *
 * To add a new bespoke scenario, use ScenarioConfig.Builder directly:
 *
 * <pre>
 *   ScenarioConfig myScenario = new ScenarioConfig.Builder("SC-099", "My test", 15.0)
 *       .vehicleSpeedKmh(80)
 *       .weatherFactor(0.6f)
 *       .addWorldObject(new WorldObject("van1", ObjectClass.VEHICLE, 50, 0, 30))
 *       .addEvent(new ScenarioEvent(5.0, ScenarioEvent.Type.TURN_RIGHT, 10.0))
 *       .addEvent(new ScenarioEvent(7.0, ScenarioEvent.Type.TURN_END, 0))
 *       .build();
 *
 *   new ScenarioGenerator(myScenario, "./scenarios/SC-099").generate();
 * </pre>
 */
public class SimulatorRunner {

    /** Root output directory */
    private static final String OUTPUT_ROOT = "./scenarios";

    public static void main(final String[] args) throws IOException {
        if (args.length == 0) {
            generateAll();
            return;
        }

        final String first = args[0];

        switch (first) {
            case "SC-001" -> generate(ScenarioLibrary.highwayCruise());
            case "SC-002" -> generate(ScenarioLibrary.highwayRearEnd());
            case "SC-003" -> generate(ScenarioLibrary.pedestrianCrossing());
            case "SC-004" -> generate(ScenarioLibrary.rainyMotorway());
            case "SC-005" -> generate(ScenarioLibrary.parkingManoeuvre());
            case "SC-006" -> generate(ScenarioLibrary.rightHandBend());
            case "SC-007" -> generate(ScenarioLibrary.inattentiveDriver());
            case "SC-008" -> generate(ScenarioLibrary.rfInterference());
            case "SC-009" -> generate(ScenarioLibrary.cyclistOvertake());
            case "SC-010" -> generate(ScenarioLibrary.emergencyStopCyclist());
            case "custom" -> {
                final String outDir = args.length > 1 ? args[1] : OUTPUT_ROOT + "/custom";
                generateCustomExample(outDir);
            }
            default -> {
                System.err.println("Unknown scenario ID: " + first);
                System.err.println("Valid IDs: SC-001 through SC-010, or 'custom'");
                System.exit(1);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private static void generateAll() throws IOException {
        System.out.println("=== Generating all scenarios ===");
        generate(ScenarioLibrary.highwayCruise());
        generate(ScenarioLibrary.highwayRearEnd());
        generate(ScenarioLibrary.pedestrianCrossing());
        generate(ScenarioLibrary.rainyMotorway());
        generate(ScenarioLibrary.parkingManoeuvre());
        generate(ScenarioLibrary.rightHandBend());
        generate(ScenarioLibrary.inattentiveDriver());
        generate(ScenarioLibrary.rfInterference());
        generate(ScenarioLibrary.cyclistOvertake());
        generate(ScenarioLibrary.emergencyStopCyclist());
        System.out.println("=== All scenarios complete ===");
    }

    private static void generate(final ScenarioConfig config) throws IOException {
        final String outDir = OUTPUT_ROOT + "/" + config.scenarioId;
        new ScenarioGenerator(config, outDir).generate();
    }

    /**
     * Demonstrates building a fully custom scenario from scratch.
     *
     * This example shows a city intersection scenario:
     *   - Vehicle approaching at 60 km/h
     *   - Pedestrian crosses at t=3s
     *   - Second vehicle approaching from right side at t=4s
     *   - Driver is late to react
     *   - Slight left turn at t=5s to model junction approach
     */
    private static void generateCustomExample(final String outDir) throws IOException {
        final ScenarioConfig custom = new ScenarioConfig.Builder(
                "SC-CUSTOM", "City intersection — crossing pedestrian and side vehicle", 10.0)
            .vehicleSpeedKmh(60.0)
            .weatherFactor(0.8f)
            .rfInterferenceFactor(0.95f)
            .sensorNoiseLevel(0.03f)
            .driverBrakes(true)
            .driverReactionTimeSec(4.0)   // late reaction

            // Pedestrian steps out from left at t=3s, 20m ahead, crossing right
            .addWorldObject(new WorldObject(
                "pedestrian1",
                WorldObject.ObjectClass.PEDESTRIAN,
                20.0, -2.5, 5.0,  // 20m ahead, 2.5m left, 5 km/h
                90.0,             // crossing left→right
                3.0,              // appears at t=3s
                Double.MAX_VALUE
            ))

            // Vehicle approaching from right (junction), visible throughout
            .addWorldObject(new WorldObject(
                "crossing_vehicle",
                WorldObject.ObjectClass.VEHICLE,
                30.0,  // 30m ahead (at the junction)
                15.0,  // 15m to the right of centreline
                40.0,  // approaching at 40 km/h
                270.0, // heading 270° = travelling left-to-right across junction
                4.0,   // appears at t=4s (enters FOV)
                Double.MAX_VALUE
            ))

            // Cyclist behind-left, overtaking
            .addWorldObject(new WorldObject(
                "cyclist1",
                WorldObject.ObjectClass.CYCLIST,
                -10.0,  // 10m behind
                -1.0,   // slightly left
                70.0,   // faster than ego
                0.0,
                0.0, Double.MAX_VALUE
            ))

            // Events
            .addEvent(new ScenarioEvent(3.0, ScenarioEvent.Type.NORMAL_BRAKE,   3.0))
            .addEvent(new ScenarioEvent(4.0, ScenarioEvent.Type.EMERGENCY_BRAKE, 8.0))
            .addEvent(new ScenarioEvent(5.0, ScenarioEvent.Type.TURN_LEFT,      3.0, 2.0))
            .addEvent(new ScenarioEvent(7.0, ScenarioEvent.Type.TURN_END,       0.0))
            .build();

        new ScenarioGenerator(custom, outDir).generate();
    }
}
