package Tester;

import swen326.group4.BrakingController;
import swen326.group4.Car.DIDInterface;
import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarReading;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor.SensorStatus;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;

/**
 * BrakingControllerTestAdapter — bridges the type gap between the test
 * framework's RadarLidarSensor-interface wrappers and BrakingController's
 * constructor, which demands concrete Radar and Lidar instances.
 *
 * ── Why this class exists ─────────────────────────────────────────────────
 *
 * BrakingController's public constructor accepts:
 *   Radar radar1/2/3  and  Lidar lidar1/2/3
 *
 * FaultInjector.FaultyRadarLidar implements only RadarLidarSensor (the shared
 * interface), not the Radar or Lidar class hierarchy, so it cannot be passed
 * directly to the real constructor.
 *
 * BrakingControllerTestAdapter solves this by accepting the already-wrapped
 * RadarLidarSensor instances and constructing thin Radar/Lidar shims whose
 * getLatestReading() and getStatus() delegate to the wrapped sensor.
 * These shims are then passed to the real BrakingController constructor.
 *
 * The shims never open any files or start any timers — they are pure delegators.
 * Lifecycle (start/stop) on shims is a no-op because the wrapped sensor
 * (real or faulty) is already started by AEBSTestRunner.
 *
 * ── Design principle ──────────────────────────────────────────────────────
 *
 * This class exists entirely in the test package and has no production usage.
 * It does not modify BrakingController in any way — all production logic is
 * preserved. The only change is at construction time.
 */
public final class BrakingControllerTestAdapter extends BrakingController {

    /**
     * Constructs the controller with wrapped sensor instances.
     *
     * @param cam1    Camera instance 1 (may be a FaultyCamera)
     * @param cam2    Camera instance 2
     * @param cam3    Camera instance 3
     * @param radar1  Wrapped radar sensor 1 (RadarLidarSensor or FaultyRadarLidar)
     * @param radar2  Wrapped radar sensor 2
     * @param radar3  Wrapped radar sensor 3
     * @param lidar1  Wrapped lidar sensor 1
     * @param lidar2  Wrapped lidar sensor 2
     * @param lidar3  Wrapped lidar sensor 3
     * @param ws1     WheelSensor 1 (may be a FaultyWheelSensor)
     * @param ws2     WheelSensor 2
     * @param ws3     WheelSensor 3
     * @param driver  Driver (may be a FaultyDriver)
     * @param chanA   Escalation channel A
     * @param chanB   Escalation channel B
     * @param did     Dashboard interface
     */
    public BrakingControllerTestAdapter(
            final Camera cam1, final Camera cam2, final Camera cam3,
            final RadarLidarSensor radar1,
            final RadarLidarSensor radar2,
            final RadarLidarSensor radar3,
            final RadarLidarSensor lidar1,
            final RadarLidarSensor lidar2,
            final RadarLidarSensor lidar3,
            final WheelSensor ws1, final WheelSensor ws2, final WheelSensor ws3,
            final Driver driver,
            final EscalationChannel chanA,
            final EscalationChannel chanB,
            final DIDInterface did) {

        super(
            cam1, cam2, cam3,
            /* Wrap each RadarLidarSensor in a thin Radar/Lidar shim so the
               concrete-typed super() constructor is satisfied. */
            shimRadar(radar1), shimRadar(radar2), shimRadar(radar3),
            shimLidar(lidar1), shimLidar(lidar2), shimLidar(lidar3),
            ws1, ws2, ws3,
            driver,
            chanA, chanB,
            did
        );
    }

    // =========================================================================
    // Shim factories
    // =========================================================================

    /**
     * Returns a Radar whose getLatestReading() and getStatus() delegate to
     * the provided RadarLidarSensor. start() and stop() are no-ops.
     */
    private static Radar shimRadar(final RadarLidarSensor wrapped) {
        return new Radar("test", "/dev/null") {
            @Override public void start()  { /* delegated */ }
            @Override public void stop()   { /* delegated */ }

            @Override
            public RadarLidarReading getLatestReading() {
                return wrapped.getLatestReading();
            }

            @Override
            public SensorStatus getStatus() {
                return wrapped.getStatus();
            }
        };
    }

    /**
     * Returns a Lidar whose getLatestReading() and getStatus() delegate to
     * the provided RadarLidarSensor. start() and stop() are no-ops.
     */
    private static Lidar shimLidar(final RadarLidarSensor wrapped) {
        return new Lidar("test", "/dev/null") {
            @Override public void start()  { /* delegated */ }
            @Override public void stop()   { /* delegated */ }

            @Override
            public RadarLidarReading getLatestReading() {
                return wrapped.getLatestReading();
            }

            @Override
            public SensorStatus getStatus() {
                return wrapped.getStatus();
            }
        };
    }
}
