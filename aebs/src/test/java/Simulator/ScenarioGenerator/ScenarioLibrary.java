package Simulator.ScenarioGenerator;

/**
 * Library of pre-built scenario configurations for common AEBS test cases.
 *
 * Each factory method returns a ScenarioConfig ready to pass to ScenarioGenerator.
 *
 * Scenarios cover:
 *   SC-001 : Highway cruise — no hazard, clear weather
 *   SC-002 : Highway rear-end — stationary truck ahead at highway speed
 *   SC-003 : Pedestrian crossing — pedestrian steps out at 50km/h
 *   SC-004 : Rainy motorway — degraded sensors, vehicle ahead braking
 *   SC-005 : 90-degree parking manoeuvre — low speed, multiple objects, turn
 *   SC-006 : Right-hand bend — vehicle drifts into path after turn
 *   SC-007 : Inattentive driver — hazard but driver never brakes
 *   SC-008 : RF interference — radar jammed, LiDAR and camera must carry vote
 *   SC-009 : Cyclist overtake — fast approaching cyclist from behind-left
 *   SC-010 : Emergency stop on dual carriageway — cyclist crosses at high speed
 */
public final class ScenarioLibrary {

    private ScenarioLibrary() {}

    // =========================================================================
    // SC-001 : Highway cruise — baseline, no hazard
    // =========================================================================

    /**
     * 10-second highway cruise at 100 km/h in clear weather.
     * No world objects. Driver does not brake.
     * Expected AEBS output: no intervention.
     */
    public static ScenarioConfig highwayCruise() {
        return new ScenarioConfig.Builder("SC-001", "Highway cruise — no hazard", 10.0)
            .vehicleSpeedKmh(100.0)
            .weatherFactor(1.0f)
            .driverBrakes(false)
            .build();
    }

    // =========================================================================
    // SC-002 : Highway rear-end — stationary truck 80m ahead
    // =========================================================================

    /**
     * Approaching a stationary truck at 100 km/h.
     * Truck is 80m ahead. Braking starts at 2s (driver reaction), but is
     * insufficient — AEBS should intervene.
     *
     * At 100 km/h = 27.78 m/s:
     *   80m / 27.78 = ~2.9 seconds to collision.
     *   Driver brakes at 2.0s — less than 1 second to act.
     */
    public static ScenarioConfig highwayRearEnd() {
        return new ScenarioConfig.Builder("SC-002", "Highway rear-end — stationary truck", 6.0)
            .vehicleSpeedKmh(100.0)
            .weatherFactor(0.95f)
            .driverBrakes(true)
            .driverReactionTimeSec(2.0)
            .addWorldObject(new WorldObject(
                "truck1",
                WorldObject.ObjectClass.VEHICLE,
                80.0,   // 80m ahead
                0.0,    // directly ahead (0 lateral offset)
                0.0     // stationary
            ))
            .addEvent(new ScenarioEvent(2.0, ScenarioEvent.Type.EMERGENCY_BRAKE, 8.0))
            .build();
    }

    // =========================================================================
    // SC-003 : Pedestrian crossing — steps out at 3 seconds
    // =========================================================================

    /**
     * Urban driving at 50 km/h. A pedestrian steps out from the left kerb
     * at t=3s, 15m ahead. Driver reacts at t=3.5s — AEBS should intervene.
     *
     * The pedestrian starts invisible (appearsAtSec=3.0) and crosses at 90°
     * heading (left-to-right across the road).
     */
    public static ScenarioConfig pedestrianCrossing() {
        return new ScenarioConfig.Builder("SC-003", "Pedestrian steps out — urban 50 km/h", 8.0)
            .vehicleSpeedKmh(50.0)
            .weatherFactor(0.9f)
            .driverBrakes(true)
            .driverReactionTimeSec(3.5)
            .addWorldObject(new WorldObject(
                "pedestrian1",
                WorldObject.ObjectClass.PEDESTRIAN,
                15.0,   // 15m ahead when they appear
                -3.5,   // starting left of centreline (stepping out from left kerb)
                5.0,    // pedestrian walking speed ~5 km/h
                90.0,   // heading 90° = crossing left-to-right
                3.0,    // becomes visible at t=3s
                Double.MAX_VALUE
            ))
            .addEvent(new ScenarioEvent(3.5, ScenarioEvent.Type.EMERGENCY_BRAKE, 9.0))
            .build();
    }

    // =========================================================================
    // SC-004 : Rainy motorway — degraded sensors
    // =========================================================================

    /**
     * Heavy rain scenario. LiDAR confidence = 0.35 (DEGRADED).
     * Camera confidence = 0.35 (DEGRADED). Radar less affected.
     * Vehicle ahead braking hard. Driver reacts normally.
     */
    public static ScenarioConfig rainyMotorway() {
        return new ScenarioConfig.Builder("SC-004", "Rainy motorway — degraded sensors", 10.0)
            .vehicleSpeedKmh(110.0)
            .weatherFactor(0.35f)
            .rfInterferenceFactor(1.0f)
            .driverBrakes(true)
            .driverReactionTimeSec(2.5)
            .addWorldObject(new WorldObject(
                "vehicle_ahead",
                WorldObject.ObjectClass.VEHICLE,
                60.0,   // 60m ahead
                0.5,    // slightly right-of-centre
                80.0    // vehicle ahead also moving, but slower
            ))
            .addEvent(new ScenarioEvent(1.0, ScenarioEvent.Type.OBJECT_SET_SPEED,
                "vehicle_ahead", 0.0))   // vehicle ahead brakes to a stop at t=1s
            .addEvent(new ScenarioEvent(2.5, ScenarioEvent.Type.EMERGENCY_BRAKE, 8.0))
            .build();
    }

    // =========================================================================
    // SC-005 : 90-degree parking manoeuvre
    // =========================================================================

    /**
     * Low-speed parking scenario at 10 km/h.
     * Multiple static objects in the scene (parked cars either side).
     * Vehicle turns right 90° at t=2s, then reverses (speed set to 5 km/h).
     * No emergency braking — tests bearing consistency through a turn.
     */
    public static ScenarioConfig parkingManoeuvre() {
        return new ScenarioConfig.Builder("SC-005", "90-degree parking — busy car park", 12.0)
            .vehicleSpeedKmh(10.0)
            .weatherFactor(1.0f)
            .driverBrakes(false)
            .addWorldObject(new WorldObject(
                "parked_left",
                WorldObject.ObjectClass.STATIONARY_OBJECT,
                5.0,    // 5m ahead
                -2.0,   // 2m left
                0.0     // stationary
            ))
            .addWorldObject(new WorldObject(
                "parked_right",
                WorldObject.ObjectClass.STATIONARY_OBJECT,
                5.0,    // 5m ahead
                2.0,    // 2m right
                0.0     // stationary
            ))
            .addWorldObject(new WorldObject(
                "wall_ahead",
                WorldObject.ObjectClass.STATIONARY_OBJECT,
                8.0,    // 8m ahead (bay end wall)
                0.0,
                0.0
            ))
            .addWorldObject(new WorldObject(
                "pedestrian_walking",
                WorldObject.ObjectClass.PEDESTRIAN,
                12.0,   // 12m ahead at start
                3.0,    // 3m right
                4.0,    // walking 4 km/h
                180.0,  // walking toward vehicle
                0.0,
                Double.MAX_VALUE
            ))
            // Begin 90° right turn at t=2s, 15 deg/sec for 6 seconds = 90°
            .addEvent(new ScenarioEvent(2.0, ScenarioEvent.Type.TURN_RIGHT, 15.0, 6.0))
            .addEvent(new ScenarioEvent(8.0, ScenarioEvent.Type.TURN_END, 0.0))
            .addEvent(new ScenarioEvent(8.0, ScenarioEvent.Type.SET_SPEED, 5.0))
            .build();
    }

    // =========================================================================
    // SC-006 : Right-hand bend — vehicle ahead drifts into path
    // =========================================================================

    /**
     * Country road at 80 km/h, right-hand bend at t=5s.
     * Vehicle ahead starts in front but drifts laterally as the ego turns.
     * Demonstrates how bearing changes through a corner.
     */
    public static ScenarioConfig rightHandBend() {
        return new ScenarioConfig.Builder("SC-006", "Right-hand bend — vehicle drift", 12.0)
            .vehicleSpeedKmh(80.0)
            .weatherFactor(0.85f)
            .driverBrakes(false)
            .addWorldObject(new WorldObject(
                "vehicle_ahead",
                WorldObject.ObjectClass.VEHICLE,
                50.0,   // 50m ahead
                0.0,
                75.0    // vehicle ahead also moving at 75 km/h
            ))
            // Begin gradual right turn at t=5s, 5 deg/sec for 10s = 50° bend
            .addEvent(new ScenarioEvent(5.0, ScenarioEvent.Type.TURN_RIGHT, 5.0))
            .addEvent(new ScenarioEvent(10.0, ScenarioEvent.Type.TURN_END, 0.0))
            .build();
    }

    // =========================================================================
    // SC-007 : Inattentive driver — driver never brakes
    // =========================================================================

    /**
     * Hazard ahead but driver does not react.
     * AEBS must act autonomously.
     * Stationaryobject (road cone / breakdown) 40m ahead at 70 km/h.
     */
    public static ScenarioConfig inattentiveDriver() {
        return new ScenarioConfig.Builder("SC-007", "Inattentive driver — no brake response", 8.0)
            .vehicleSpeedKmh(70.0)
            .weatherFactor(0.95f)
            .driverBrakes(false)    // driver never brakes
            .addWorldObject(new WorldObject(
                "road_cone",
                WorldObject.ObjectClass.STATIONARY_OBJECT,
                40.0,   // 40m ahead
                0.0,
                0.0
            ))
            .build();
    }

    // =========================================================================
    // SC-008 : RF interference — radar jammed
    // =========================================================================

    /**
     * Radar is heavily jammed (rfInterferenceFactor = 0.08 → FAILED).
     * LiDAR and camera must carry the 2oo3 vote for radar; camera 2oo3 unaffected.
     * Vehicle ahead 70m, driver brakes normally.
     */
    public static ScenarioConfig rfInterference() {
        return new ScenarioConfig.Builder("SC-008", "Radar jammed — RF interference", 8.0)
            .vehicleSpeedKmh(90.0)
            .weatherFactor(0.95f)
            .rfInterferenceFactor(0.05f)    // radar FAILED
            .driverBrakes(true)
            .driverReactionTimeSec(2.0)
            .addWorldObject(new WorldObject(
                "vehicle_stopped",
                WorldObject.ObjectClass.VEHICLE,
                70.0,
                0.0,
                0.0
            ))
            .addEvent(new ScenarioEvent(2.0, ScenarioEvent.Type.EMERGENCY_BRAKE, 8.0))
            .build();
    }

    // =========================================================================
    // SC-009 : Cyclist overtake — approaching from behind-left
    // =========================================================================

    /**
     * Vehicle is changing lanes right. A cyclist approaches from behind-left
     * at 30 km/h faster. Cyclist appears at t=2s, 20m behind-left.
     *
     * distanceAhead = -20 (behind), lateral = -1.5m (left of centreline)
     * Cyclist heading = 0 (same direction as ego) at speed ego+30.
     */
    public static ScenarioConfig cyclistOvertake() {
        final double egoSpeed = 50.0;
        return new ScenarioConfig.Builder("SC-009", "Cyclist overtake — lane change hazard", 10.0)
            .vehicleSpeedKmh(egoSpeed)
            .weatherFactor(0.9f)
            .driverBrakes(false)
            .addWorldObject(new WorldObject(
                "cyclist",
                WorldObject.ObjectClass.CYCLIST,
                -20.0,          // 20m behind ego at start
                -1.5,           // 1.5m left of centreline
                egoSpeed + 30,  // overtaking at 30 km/h faster
                0.0,            // same heading as ego
                2.0,            // appears at t=2s
                Double.MAX_VALUE
            ))
            // Ego starts drifting right at t=3s (lane change)
            .addEvent(new ScenarioEvent(3.0, ScenarioEvent.Type.TURN_RIGHT, 2.0))
            .addEvent(new ScenarioEvent(5.0, ScenarioEvent.Type.TURN_END,   0.0))
            .build();
    }

    // =========================================================================
    // SC-010 : Emergency stop — head-on crossing cyclist
    // =========================================================================

    /**
     * High-speed dual carriageway. Cyclist crosses the road head-on
     * (heading = 90°) from 80m ahead at t=0. Very short TTC.
     * Driver brakes very late (3s). AEBS must intervene earlier.
     */
    public static ScenarioConfig emergencyStopCyclist() {
        return new ScenarioConfig.Builder("SC-010",
                "Emergency stop — cyclist crosses dual carriageway", 6.0)
            .vehicleSpeedKmh(100.0)
            .weatherFactor(0.9f)
            .driverBrakes(true)
            .driverReactionTimeSec(3.0)
            .addWorldObject(new WorldObject(
                "cyclist",
                WorldObject.ObjectClass.CYCLIST,
                80.0,   // 80m ahead
                -5.0,   // starts on left hard shoulder
                25.0,   // fast cyclist 25 km/h
                90.0,   // crossing left to right
                0.0,
                Double.MAX_VALUE
            ))
            .addEvent(new ScenarioEvent(3.0, ScenarioEvent.Type.EMERGENCY_BRAKE, 10.0))
            .build();
    }
}
