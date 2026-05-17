package Tester;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Camera.CameraReading;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Driver.DriverReading;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarReading;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensorData;

import java.util.List;
import java.util.Collections;

/**
 * FaultInjector — thin wrapper classes that override sensor behaviour for testing.
 *
 * Each inner class implements the same public interface as the real sensor so it
 * can be dropped in transparently wherever the real sensor is used. Faults can be
 * applied statically (always broken) or dynamically (broken after N ticks).
 *
 * Fault types supported:
 *   HARD_FAIL          — sensor returns null readings and FAILED status immediately.
 *   SILENT             — sensor produces no data (returns null) but status stays OK.
 *   FROZEN_READING     — sensor returns the same reading forever (stuck value).
 *   DEGRADED           — sensor reports DEGRADED status but still produces data.
 *   DELAYED_FAIL       — sensor works normally for delayTicks, then hard-fails.
 *   INTERMITTENT       — sensor alternates between valid and null each tick.
 *
 * Design rationale:
 *   Real sensors read scenario JSON files, making it impractical to inject faults
 *   at the file level for every test. These wrappers intercept the interface layer
 *   instead, matching the 2oo3 architecture boundary used by BrakingController voters.
 *
 * Requirement traceability:
 *   FR2104  : HARD_FAIL and DELAYED_FAIL verify FAILED sensors are excluded from vote.
 *   FR2103  : DEGRADED verifies controller shifts weight to higher-confidence sensor.
 *   FR2208  : All radar/lidar/wheel faults verify the 2oo3 architecture holds.
 *   NF2202  : Camera SILENT and FROZEN test fallback and insufficient-camera paths.
 */
public final class FaultInjector {

    /** The set of injectable fault modes. */
    public enum FaultType {
        NONE,
        HARD_FAIL,
        SILENT,
        FROZEN_READING,
        DEGRADED,
        DELAYED_FAIL,
        INTERMITTENT
    }

    // =========================================================================
    // FaultyCamera
    // =========================================================================

    /**
     * A Camera substitute that delegates to a real Camera instance but overrides
     * getLatestReading(), getStatus(), isVotingEligible(), start(), and stop()
     * according to the configured FaultType.
     *
     * The real sensor is constructed but start() is only forwarded when NONE
     * (no fault). For all fault modes the clock is never started on the real
     * sensor, so no file reads occur — this keeps tests file-independent.
     */
    public static final class FaultyCamera extends Camera {

        private final FaultType faultType;
        private final int       delayTicks;
        private int             tickCount;

        /**
         * Constructs a FaultyCamera.
         *
         * @param sensorId      "1", "2", or "3" — passed to the real Camera super
         * @param dataDirectory scenario data directory — passed to super
         * @param faultType     which fault to inject
         * @param delayTicks    ticks before fault activates (only used by DELAYED_FAIL)
         */
        public FaultyCamera(final String sensorId,
                            final String dataDirectory,
                            final FaultType faultType,
                            final int delayTicks) {
            super(sensorId, dataDirectory);
            this.faultType  = faultType;
            this.delayTicks = delayTicks;
            this.tickCount  = 0;
        }

        /** Convenience constructor for faults that need no delay count. */
        public FaultyCamera(final String sensorId,
                            final String dataDirectory,
                            final FaultType faultType) {
            this(sensorId, dataDirectory, faultType, 0);
        }

        @Override
        public void start() {
            // Only start the real sensor (and its file reader) for fault-free cameras.
            if (faultType == FaultType.NONE) {
                super.start();
            }
            // All fault modes: do NOT start the underlying timer/file reader —
            // the fault wrapper handles all output.
        }

        @Override
        public void stop() {
            if (faultType == FaultType.NONE) {
                super.stop();
            }
        }

        @Override
        public CameraReading getLatestReading() {
            tickCount++;
            switch (faultType) {
                case NONE:
                    return super.getLatestReading();

                case HARD_FAIL:
                case SILENT:
                    return null;

                case FROZEN_READING:
                    return buildFrozenCameraReading();

                case DEGRADED:
                    return buildFrozenCameraReading(); // data present, status degraded

                case DELAYED_FAIL:
                    return (tickCount > delayTicks) ? null : buildFrozenCameraReading();

                case INTERMITTENT:
                    return (tickCount % 2 == 0) ? null : buildFrozenCameraReading();

                default:
                    return null;
            }
        }

        @Override
        public SensorStatus getStatus() {
            switch (faultType) {
                case NONE:           return super.getStatus();
                case HARD_FAIL:      return SensorStatus.FAILED;
                case SILENT:         return SensorStatus.FAILED;
                case FROZEN_READING: return SensorStatus.OK;
                case DEGRADED:       return SensorStatus.DEGRADED;
                case DELAYED_FAIL:   return (tickCount > delayTicks)
                                        ? SensorStatus.FAILED : SensorStatus.OK;
                case INTERMITTENT:   return SensorStatus.OK;
                default:             return SensorStatus.FAILED;
            }
        }

        @Override
        public boolean isVotingEligible() {
            final SensorStatus s = getStatus();
            return s == SensorStatus.OK || s == SensorStatus.DEGRADED;
        }

        /** Returns a minimal but valid CameraReading (VEHICLE at bearing 0). */
        private CameraReading buildFrozenCameraReading() {
            final Camera.ObjectType[] types = new Camera.ObjectType[Camera.MAX_OBJECTS];
            final float[] scores   = new float[Camera.MAX_OBJECTS];
            final float[] bearings = new float[Camera.MAX_OBJECTS];
            for (int i = 0; i < Camera.MAX_OBJECTS; i++) {
                types[i]   = Camera.ObjectType.NONE;
                scores[i]  = 0.0f;
                bearings[i]= 0.0f;
            }
            types[0]   = Camera.ObjectType.VEHICLE;
            scores[0]  = 0.9f;
            bearings[0]= 1.5f;
            return new CameraReading(types, scores, bearings, 1, System.currentTimeMillis());
        }
    }

    // =========================================================================
    // FaultyRadarLidar — shared wrapper for Radar and Lidar (both implement
    // RadarLidarSensor, so a single wrapper covers both).
    // =========================================================================

    /**
     * Implements RadarLidarSensor and delegates to either a real Radar or a real
     * Lidar instance, overriding readings and status according to FaultType.
     *
     * A label ("Radar" or "Lidar") is accepted for logging only.
     */
    public static final class FaultyRadarLidar implements RadarLidarSensor {

        private final RadarLidarSensor delegate;
        private final FaultType        faultType;
        private final int              delayTicks;
        private final String           label;
        private int                    tickCount;

        /**
         * Constructs a FaultyRadarLidar wrapping a real sensor.
         *
         * @param delegate   the real Radar or Lidar instance
         * @param faultType  which fault to inject
         * @param delayTicks ticks before DELAYED_FAIL activates
         * @param label      "Radar" or "Lidar" — for console logging
         */
        public FaultyRadarLidar(final RadarLidarSensor delegate,
                                 final FaultType faultType,
                                 final int delayTicks,
                                 final String label) {
            assert delegate != null : "delegate must not be null";
            this.delegate   = delegate;
            this.faultType  = faultType;
            this.delayTicks = delayTicks;
            this.label      = label;
            this.tickCount  = 0;
        }

        /** Convenience constructor for faults with no delay. */
        public FaultyRadarLidar(final RadarLidarSensor delegate,
                                 final FaultType faultType,
                                 final String label) {
            this(delegate, faultType, 0, label);
        }

        @Override
        public void start() {
            if (faultType == FaultType.NONE) {
                delegate.start();
            }
        }

        @Override
        public void stop() {
            if (faultType == FaultType.NONE) {
                delegate.stop();
            }
        }

        @Override
        public RadarLidarReading getLatestReading() {
            tickCount++;
            switch (faultType) {
                case NONE:
                    return delegate.getLatestReading();

                case HARD_FAIL:
                case SILENT:
                    return null;

                case FROZEN_READING:
                    return buildFrozenReading(1.0f);

                case DEGRADED:
                    return buildFrozenReading(0.4f); // low confidence reading

                case DELAYED_FAIL:
                    return (tickCount > delayTicks) ? null : buildFrozenReading(1.0f);

                case INTERMITTENT:
                    return (tickCount % 2 == 0) ? null : buildFrozenReading(1.0f);

                default:
                    return null;
            }
        }

        @Override
        public SensorStatus getStatus() {
            switch (faultType) {
                case NONE:           return delegate.getStatus();
                case HARD_FAIL:      return SensorStatus.FAILED;
                case SILENT:         return SensorStatus.FAILED;
                case FROZEN_READING: return SensorStatus.OK;
                case DEGRADED:       return SensorStatus.DEGRADED;
                case DELAYED_FAIL:   return (tickCount > delayTicks)
                                        ? SensorStatus.FAILED : SensorStatus.OK;
                case INTERMITTENT:   return SensorStatus.OK;
                default:             return SensorStatus.FAILED;
            }
        }

        /** Returns a fixed-distance reading (50m, approaching at -30 km/h). */
        private RadarLidarReading buildFrozenReading(final float confidence) {
            swen326.group4.Sensors.Radar_Lidar.DetectedObject obj =
                new swen326.group4.Sensors.Radar_Lidar.DetectedObject(50.0f, -30.0f, 1.5f);
            return new RadarLidarReading(
                Collections.singletonList(obj),
                confidence,
                System.currentTimeMillis()
            );
        }
    }

    // =========================================================================
    // FaultyWheelSensor
    // =========================================================================

    /**
     * Wraps a real WheelSensor and overrides getLatestReading() according to
     * FaultType. Used to test wheel-voter fault tolerance (FR2208).
     */
    public static final class FaultyWheelSensor extends WheelSensor {

        private final FaultType faultType;
        private final int       delayTicks;
        private int             tickCount;

        /**
         * @param sensorId      "1", "2", or "3"
         * @param dataDirectory scenario data directory
         * @param faultType     which fault to inject
         * @param delayTicks    ticks before DELAYED_FAIL activates
         */
        public FaultyWheelSensor(final String sensorId,
                                  final String dataDirectory,
                                  final FaultType faultType,
                                  final int delayTicks) {
            super(sensorId, dataDirectory);
            this.faultType  = faultType;
            this.delayTicks = delayTicks;
            this.tickCount  = 0;
        }

        public FaultyWheelSensor(final String sensorId,
                                  final String dataDirectory,
                                  final FaultType faultType) {
            this(sensorId, dataDirectory, faultType, 0);
        }

        @Override
        public void start() {
            if (faultType == FaultType.NONE) {
                super.start();
            }
        }

        @Override
        public void stop() {
            if (faultType == FaultType.NONE) {
                super.stop();
            }
        }

        @Override
        public WheelSensorData getLatestReading() {
            tickCount++;
            switch (faultType) {
                case NONE:
                    return super.getLatestReading();

                case HARD_FAIL:
                case SILENT:
                    return null;

                case FROZEN_READING:
                    return buildFrozenWheelReading();

                case DEGRADED:
                    return buildFrozenWheelReading();

                case DELAYED_FAIL:
                    return (tickCount > delayTicks) ? null : buildFrozenWheelReading();

                case INTERMITTENT:
                    return (tickCount % 2 == 0) ? null : buildFrozenWheelReading();

                default:
                    return null;
            }
        }

        /** Returns a valid reading at 1120 RPM across all four wheels. */
        private WheelSensorData buildFrozenWheelReading() {
            return new WheelSensorData(
                tickCount * 10,
                new float[]{ 1120.0f, 1120.0f, 1120.0f, 1120.0f },
                Integer.parseInt(getSensorId() + "")
            );
        }

        /** Expose sensorId as a String so buildFrozenWheelReading can use it. */
        private String getSensorIdStr() {
            // WheelSensor exposes getSensorId() as int — call it here
            return String.valueOf(super.getSensorId());
        }
    }

    // =========================================================================
    // FaultyDriver
    // =========================================================================

    /**
     * Wraps a real Driver instance and overrides getLatestReading() to simulate
     * driver fault scenarios:
     *
     *   HARD_FAIL   — driver sensor offline; returns null.
     *   SILENT      — driver never brakes (always NONE). Tests FR3104 path where
     *                 manual braking is never detected even under hazard.
     *   FROZEN_READING — driver brakes on every tick. Tests over-eager brake path.
     */
    public static final class FaultyDriver extends Driver {

        private final FaultType faultType;

        public FaultyDriver(final String dataDirectory, final FaultType faultType) {
            super(dataDirectory);
            this.faultType = faultType;
        }

        @Override
        public void start() {
            if (faultType == FaultType.NONE) {
                super.start();
            }
        }

        @Override
        public void stop() {
            if (faultType == FaultType.NONE) {
                super.stop();
            }
        }

        @Override
        public DriverReading getLatestReading() {
            switch (faultType) {
                case NONE:
                    return super.getLatestReading();
                case HARD_FAIL:
                case SILENT:
                    return new DriverReading(Driver.Action.NONE, System.currentTimeMillis());
                case FROZEN_READING:
                    return new DriverReading(Driver.Action.BRAKE, System.currentTimeMillis());
                default:
                    return null;
            }
        }
    }
}
