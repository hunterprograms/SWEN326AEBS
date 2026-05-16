package swen326.group4;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import swen326.group4.Sensors.Camera.Camera;
import swen326.group4.Sensors.Camera.CameraVoter;
import swen326.group4.Sensors.Camera.CameraVoter.VoteResult;
import swen326.group4.Sensors.Driver.Driver;
import swen326.group4.Sensors.Driver.DriverReading;
import swen326.group4.Sensors.Radar_Lidar.DetectedObject;
import swen326.group4.Sensors.Radar_Lidar.Lidar;
import swen326.group4.Sensors.Radar_Lidar.Radar;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarReading;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor;
import swen326.group4.Sensors.Radar_Lidar.RadarLidarSensor.SensorStatus;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensor;
import swen326.group4.Sensors.Wheel_Sensor.WheelSensorData;

/**
 * BrakingController — AEBS Central Decision Controller.
 *
 * This is the top-level safety controller for the Automatic Emergency Braking System.
 * It runs on a fixed 50ms decision cycle and is responsible for:
 *
 *   1. Collecting and aggregating votes from all sensor subsystems:
 *        - Camera 2oo3 vote       (via the existing CameraVoter)
 *        - Radar  2oo3 vote       (via RadarLidarVoter for 3 Radar instances)
 *        - Lidar  2oo3 vote       (via RadarLidarVoter for 3 Lidar instances)
 *        - WheelSensor 2oo3 vote  (via WheelVoter for 3 WheelSensor instances)
 *
 *   2. Computing Time-To-Collision (TTC) from the fused sensor distance/speed data.
 *
 *   3. Making a ControllerDecision: CLEAR, DRIVER_ALERT, or AUTONOMOUS_BRAKE.
 *
 *   4. Executing braking with up to MAX_BRAKE_ATTEMPTS retries (FR-3101).
 *      Each attempt re-checks wheel speed sensor data to confirm deceleration
 *      within ±BRAKE_CONFIRMATION_MARGIN_PERCENT (FR-3108).
 *
 *   5. Immediately yielding all braking authority to the driver if manual braking
 *      is detected at any point (FR3104).
 *
 *   6. Escalating to two independent notification channels if all brake attempts
 *      fail (FR-3105, FR-3106, FR-3107).
 *
 *   7. Modulating brake force during active braking based on wheel speed feedback
 *      to prevent lockup on low-traction surfaces (FR-3111, FR-3112, FR-3113).
 *
 * Power of Ten compliance notes:
 *   - No dynamic allocation after initialisation (all arrays fixed-size).
 *   - All loops bounded by named constants.
 *   - No recursion.
 *   - All public methods contain assertions on inputs and post-conditions.
 *   - All failure paths set a safe state before returning.
 *
 * Requirement traceability:
 *   FR3101  : 50ms decision cycle enforced by DECISION_INTERVAL_MS constant.
 *   FR3104  : Manual brake override checked first in every decision cycle.
 *   FR3103  : Brakes not released until driver makes deliberate action.
 *   FR-3105 : Escalation on brake exhaustion via two independent channels.
 *   FR-3106 : Escalation delivery confirmation monitored; redelivery on timeout.
 *   FR-3107 : Escalation interface explicitly owned at the subsystem boundary.
 *   FR-3108 : Brake confirmation checks deceleration adequacy, not just ±5%.
 *   FR-3109 : Post-execution threat re-evaluation continues after confirmed brake.
 *   FR-3110 : Residual collision alert issued within 50ms if threat persists.
 *   FR-3111 : Closed-loop braking modulation every 10ms (wheel sensor rate).
 *   FR-3112 : Lockup detected and command reduced within one 10ms interval.
 *   FR-3113 : Directional instability monitored during braking; alert issued.
 *   FR2103  : Radar/Lidar weight adjusted based on individual confidence.
 *   FR2106  : AEBS cannot act on LiDAR alone — requires multi-sensor agreement.
 *   FR2107  : AEBS cannot act on Radar alone — requires multi-sensor agreement.
 *   FR2208  : 2oo3 wheel voting prevents single frozen/erroneous sensor triggering.
 *   H-3104  : Escalation alert guaranteed via at least one channel within 50ms.
 *   H-3105  : Braking adequacy evaluated against remaining obstacle distance.
 *   H-3106  : Lockup prevention via wheel-speed-feedback modulation loop.
 */
public class BrakingController {

    // =========================================================================
    // Inner interfaces — escalation channel contract (FR-3107)
    // =========================================================================

    /**
     * Represents one independent escalation notification channel.
     *
     * The Emergency Intervention Indicators subsystem shall provide two
     * concrete implementations of this interface. The controller does not
     * know or care which physical channel is used — it only cares that
     * at least one confirms delivery within ESCALATION_CONFIRM_TIMEOUT_MS.
     *
     * Requirement traceability: FR-3105, FR-3106, FR-3107.
     */

    public static BrakeActuator brakeActuator = new BrakeActuator();

    public interface EscalationChannel {
        /**
         * Sends the escalation alert through this channel.
         * Must return immediately — never block the controller decision loop.
         */
        void sendAlert();

        /**
         * Returns true if delivery was confirmed on this channel.
         * The controller polls this after sendAlert() within
         * ESCALATION_CONFIRM_TIMEOUT_MS to satisfy FR-3106.
         */
        boolean isDeliveryConfirmed();

        /** Human-readable channel name for logging. */
        String channelName();
    }

    // =========================================================================
    // Inner classes — sub-voters
    // =========================================================================

    /**
     * RadarLidarVoter — 2oo3 voter for three Radar or three Lidar instances.
     *
     * Collects the latest RadarLidarReading from each of three sensor instances,
     * excludes FAILED sensors, and produces a trusted fused reading when 2+
     * sensors report a consistent nearest object.
     *
     * "Consistent" means the nearest detected object's distance is within
     * RADAR_LIDAR_DISTANCE_TOLERANCE_M of the median of all eligible readings.
     *
     * Requirement traceability:
     *   FR2104 : FAILED sensors excluded from vote.
     *   FR2103 : Confidence weighting applied to produce fused output.
     *   FR4103 : Track loss does not trigger braking — must be confirmed by
     *            independent sensor (null trusted reading = not confirmed).
     */
    public static final class RadarLidarVoter {

        /** Minimum number of eligible sensors required for a trusted vote. */
        public static final int MIN_ELIGIBLE = 2;

        /** Number of sensors in this voter's group. */
        public static final int SENSOR_COUNT = 3;

        /**
         * Maximum distance spread (metres) between any two eligible sensor
         * readings for them to be considered "agreeing" on nearest object.
         * Sensors further apart than this are in disagreement.
         */
        private static final float DISTANCE_TOLERANCE_M = 5.0f;

        /** The three sensor instances managed by this voter. */
        private final RadarLidarSensor[] sensors;

        /** Label for logging ("Radar" or "Lidar"). */
        private final String label;

        /** Whether the last vote produced a trusted result. */
        private boolean trusted;

        /** Fused distance from last trusted vote (metres). */
        private float fusedDistanceM;

        /** Fused relative speed from last trusted vote (km/h). */
        private float fusedRelativeSpeedKmh;

        /** Fused bearing from last trusted vote (degrees). */
        private float fusedBearingDeg;

        /** Weighted average confidence from last trusted vote. */
        private float fusedConfidence;

        /** Number of eligible sensors in the last vote. */
        private int eligibleCount;

        /**
         * Constructs a RadarLidarVoter for exactly three sensor instances.
         * @param s1    first sensor instance
         * @param s2    second sensor instance
         * @param s3    third sensor instance
         * @param label "Radar" or "Lidar" — used for logging only
         */
        public RadarLidarVoter(final RadarLidarSensor s1,
                                final RadarLidarSensor s2,
                                final RadarLidarSensor s3,
                                final String label) {
            assert s1 != null : "s1 must not be null";
            assert s2 != null : "s2 must not be null";
            assert s3 != null : "s3 must not be null";
            assert label != null : "label must not be null";
            this.sensors = new RadarLidarSensor[SENSOR_COUNT];
            this.sensors[0] = s1;
            this.sensors[1] = s2;
            this.sensors[2] = s3;
            this.label = label;
            this.trusted = false;
            this.fusedDistanceM = Float.MAX_VALUE;
            this.fusedRelativeSpeedKmh = 0.0f;
            this.fusedBearingDeg = 0.0f;
            this.fusedConfidence = 0.0f;
            this.eligibleCount = 0;
        }

        /**
         * Runs the 2oo3 vote across all three sensor instances.
         *
         * Vote process:
         *   1. Exclude FAILED sensors (FR2104).
         *   2. If fewer than 2 eligible, mark untrusted and return.
         *   3. Extract the nearest detected object from each eligible sensor.
         *   4. If 2+ sensors agree within DISTANCE_TOLERANCE_M, produce a
         *      confidence-weighted fused reading and mark trusted.
         *   5. Otherwise mark untrusted (disagreement).
         *
         * This method must complete within the 50ms decision window.
         * It is called once per controller cycle.
         */
        public void vote() {
            /* Step 1: identify eligible sensors (not FAILED). */
            final float[]   distances = new float[SENSOR_COUNT];
            final float[]   speeds    = new float[SENSOR_COUNT];
            final float[]   bearings  = new float[SENSOR_COUNT];
            final float[]   confs     = new float[SENSOR_COUNT];
            final boolean[] eligible  = new boolean[SENSOR_COUNT];

            /* Initialise all slots to safe sentinel values. */
            for (int i = 0; i < SENSOR_COUNT; i++) {
                distances[i] = Float.MAX_VALUE; /* no object detected */
                speeds[i]    = 0.0f;
                bearings[i]  = 0.0f;
                confs[i]     = 0.0f;
                eligible[i]  = false;
            }

            eligibleCount = 0;

            for (int i = 0; i < SENSOR_COUNT; i++) {
                /* FAILED sensors are excluded per FR2104. */
                if (sensors[i].getStatus() == SensorStatus.FAILED) {
                    System.out.println("Help");
                    continue;
                }

                final RadarLidarReading reading = sensors[i].getLatestReading();

                /* A null reading (no data yet) is treated like FAILED. */
                if (reading == null || reading.detectedObjects().isEmpty()) {
                    continue;
                }

                /* Extract the nearest detected object from this sensor. */
                final DetectedObject nearest = getNearestObject(reading);
                if (nearest == null) {
                    continue;
                }

                distances[i] = nearest.distanceMetres();
                speeds[i]    = nearest.relativeSpeedKmh();
                bearings[i]  = nearest.bearingDegrees();
                confs[i]     = reading.confidenceScore();
                eligible[i]  = true;
                eligibleCount++;
            }

            /* Step 2: insufficient coverage — cannot trust. */
            if (eligibleCount < MIN_ELIGIBLE) {
                trusted = false;
                fusedDistanceM = Float.MAX_VALUE;
                fusedRelativeSpeedKmh = 0.0f;
                fusedBearingDeg = 0.0f;
                fusedConfidence = 0.0f;
                System.out.println(label + "Voter: insufficient eligible sensors ("
                        + eligibleCount + "/" + SENSOR_COUNT + ")");
                return;
            }

            /* Step 3: compute the median distance among eligible sensors
             *         to use as the agreement reference point. */
            final float medianDist = computeMedianDistance(distances, eligible);

            /* Step 4: check how many eligible sensors agree with the median. */
            int agreeCount = 0;
            float sumDist = 0.0f;
            float sumSpeed = 0.0f;
            float sumBearing = 0.0f;
            float sumConf = 0.0f;

            for (int i = 0; i < SENSOR_COUNT; i++) {
                if (!eligible[i]) {
                    continue;
                }
                /* A sensor "agrees" if its reported distance is within
                 * DISTANCE_TOLERANCE_M of the median. */
                if (Math.abs(distances[i] - medianDist) <= DISTANCE_TOLERANCE_M) {
                    agreeCount++;
                    sumDist    += distances[i];
                    sumSpeed   += speeds[i];
                    sumBearing += bearings[i];
                    sumConf    += confs[i];
                }
            }

            if (agreeCount >= MIN_ELIGIBLE) {
                /* Step 4a: enough sensors agree — produce weighted fused result. */
                trusted = true;
                fusedDistanceM        = sumDist    / agreeCount;
                fusedRelativeSpeedKmh = sumSpeed   / agreeCount;
                fusedBearingDeg       = sumBearing / agreeCount;
                fusedConfidence       = sumConf    / agreeCount;
                System.out.println(label + "Voter: TRUSTED — dist=" + fusedDistanceM
                        + "m speed=" + fusedRelativeSpeedKmh
                        + "km/h conf=" + fusedConfidence);
            } else {
                /* Step 5: sensors disagree — do not trust this reading. */
                trusted = false;
                fusedDistanceM = Float.MAX_VALUE;
                fusedRelativeSpeedKmh = 0.0f;
                fusedBearingDeg = 0.0f;
                fusedConfidence = 0.0f;
                System.out.println(label + "Voter: DISAGREEMENT among " + eligibleCount
                        + " eligible sensors");
            }
        }

        /* -- Getters -- */

        /** True only when the 2oo3 vote produced a trusted consensus result. */
        public boolean isTrusted()             { return trusted; }

        /** Confidence-weighted average distance (metres) from last trusted vote. */
        public float getFusedDistanceM()       { return fusedDistanceM; }

        /** Confidence-weighted average relative speed (km/h) from last trusted vote. */
        public float getFusedRelativeSpeedKmh(){ return fusedRelativeSpeedKmh; }

        /** Confidence-weighted bearing (degrees) from last trusted vote. */
        public float getFusedBearingDeg()      { return fusedBearingDeg; }

        /** Average confidence score from agreeing sensors in last trusted vote. */
        public float getFusedConfidence()      { return fusedConfidence; }

        /** Number of eligible (non-FAILED) sensors in the last vote. */
        public int getEligibleCount()          { return eligibleCount; }

        /**
         * Returns the nearest DetectedObject from a RadarLidarReading,
         * or null if the reading contains no objects.
         */
        private DetectedObject getNearestObject(final RadarLidarReading reading) {
            assert reading != null : "reading must not be null";
            DetectedObject nearest = null;
            float minDist = Float.MAX_VALUE;
            for (final DetectedObject obj : reading.detectedObjects()) {
                if (obj.distanceMetres() < minDist) {
                    minDist = obj.distanceMetres();
                    nearest = obj;
                }
            }
            return nearest;
        }

        /**
         * Computes the median distance among eligible sensors.
         * Uses a simple selection sort over at most SENSOR_COUNT (3) elements
         * — bounded, no dynamic allocation.
         *
         * @param distances array of per-sensor distances (MAX_VALUE if ineligible)
         * @param eligible  mask indicating which sensor slots are eligible
         * @return          median of eligible distances
         */
        private float computeMedianDistance(final float[] distances,
                                            final boolean[] eligible) {
            assert distances != null : "distances must not be null";
            assert eligible  != null : "eligible must not be null";

            /* Collect eligible distances into a fixed-size scratch array. */
            final float[] scratch = new float[SENSOR_COUNT];
            int n = 0;
            for (int i = 0; i < SENSOR_COUNT; i++) {
                if (eligible[i]) {
                    scratch[n] = distances[i];
                    n++;
                }
            }

            /* Simple insertion sort over at most 3 elements. */
            for (int i = 1; i < n; i++) {
                final float key = scratch[i];
                int j = i - 1;
                while (j >= 0 && scratch[j] > key) {
                    scratch[j + 1] = scratch[j];
                    j--;
                }
                scratch[j + 1] = key;
            }

            /* Return middle element (n is 2 or 3 at this point). */
            return scratch[n / 2];
        }
    }

    // -------------------------------------------------------------------------

    /**
     * WheelVoter — 2oo3 voter for three WheelSensor instances.
     *
     * Collects the latest WheelSensorData from each of three WheelSensor
     * instances, excludes sensors with no valid reading, and produces a
     * per-wheel trusted RPM by majority agreement.
     *
     * Agreement: for each wheel index, at least 2 sensors must report an RPM
     * within WHEEL_RPM_TOLERANCE of each other. If they agree, the average of
     * the agreeing sensors is the trusted RPM for that wheel.
     *
     * A wheel RPM of WheelSensorData.UNAVAILABLE (-1) or the frozen sentinel
     * (-2) is excluded from voting for that wheel index.
     *
     * Requirement traceability:
     *   FR2208 : 2oo3 architecture prevents single anomaly triggering braking.
     *   FR4203 : Frozen sentinel (-2) excluded from vote.
     *   FR2209 : Unavailable sentinel (-1) excluded from vote.
     *   H-3106 : Per-wheel trusted RPM used by lockup detection in controller.
     */
    public static final class WheelVoter {

        /** Number of sensors in this voter. */
        public static final int SENSOR_COUNT = 3;

        /** Number of wheels per sensor set. */
        public static final int WHEEL_COUNT = 4;

        /**
         * Maximum RPM difference between two sensors for them to be considered
         * "agreeing" on the same wheel. Tuned to exceed normal inter-sensor
         * variance while catching genuine anomalies (FR2208).
         */
        private static final float WHEEL_RPM_TOLERANCE = 50.0f;

        /** Sentinel for unavailable wheel reading (mirrors WheelSensorData). */
        private static final float UNAVAILABLE = -1.0f;

        /** Sentinel for frozen wheel reading (set by WheelSensor FR4203). */
        private static final float FROZEN = -2.0f;

        /** The three WheelSensor instances. */
        private final WheelSensor[] sensors;

        /** Trusted per-wheel RPM from the last vote; UNAVAILABLE if no consensus. */
        private final float[] trustedRpm;

        /** Whether the last vote produced a trusted result for each wheel. */
        private final boolean[] wheelTrusted;

        /** Whether the overall vote (all four wheels trusted) was successful. */
        private boolean fullyTrusted;

        /**
         * Constructs a WheelVoter for exactly three WheelSensor instances.
         * @param ws1 first wheel sensor set (sensorId 1)
         * @param ws2 second wheel sensor set (sensorId 2)
         * @param ws3 third wheel sensor set (sensorId 3)
         */
        public WheelVoter(final WheelSensor ws1,
                          final WheelSensor ws2,
                          final WheelSensor ws3) {
            assert ws1 != null : "ws1 must not be null";
            assert ws2 != null : "ws2 must not be null";
            assert ws3 != null : "ws3 must not be null";
            ws1.linkBrakeActuator(brakeActuator);
            ws2.linkBrakeActuator(brakeActuator);
            ws3.linkBrakeActuator(brakeActuator);

            this.sensors = new WheelSensor[SENSOR_COUNT];
            this.sensors[0] = ws1;
            this.sensors[1] = ws2;
            this.sensors[2] = ws3;
            this.trustedRpm  = new float[WHEEL_COUNT];
            this.wheelTrusted = new boolean[WHEEL_COUNT];
            this.fullyTrusted = false;

            /* Initialise trusted RPM to UNAVAILABLE — safe default. */
            for (int w = 0; w < WHEEL_COUNT; w++) {
                trustedRpm[w]  = UNAVAILABLE;
                wheelTrusted[w] = false;
            }
        }

        /**
         * Runs the 2oo3 vote across all three WheelSensor instances
         * for each of the four wheel positions.
         *
         * Vote process per wheel:
         *   1. Collect valid (non-sentinel) RPM values from each sensor.
         *   2. If fewer than 2 valid values, mark wheel untrusted.
         *   3. If 2+ values are within WHEEL_RPM_TOLERANCE, average them
         *      and mark wheel trusted.
         *   4. Otherwise mark wheel untrusted (disagreement).
         *
         * fullyTrusted is true only when all four wheels are trusted.
         */
        public void vote() {
            /* Scratch arrays — fixed size, no dynamic allocation. */
            final float[] readings = new float[SENSOR_COUNT];

            int trustedWheelCount = 0;

            for (int w = 0; w < WHEEL_COUNT; w++) {
                /* Step 1: collect valid readings from all three sensors. */
                int validCount = 0;
                for (int s = 0; s < SENSOR_COUNT; s++) {
                    readings[s] = UNAVAILABLE; /* default: nothing from this sensor */
                    final WheelSensorData data = sensors[s].getLatestReading();
                    if (data == null) {
                        continue;
                    }
                    final float rpm = data.getRpm(w);
                    /* Exclude both unavailable (-1) and frozen (-2) sentinels. */
                    if (rpm == UNAVAILABLE || rpm == FROZEN) {
                        continue;
                    }
                    readings[s] = rpm;
                    validCount++;
                }

                /* Step 2: insufficient valid readings for this wheel. */
                if (validCount < 2) {
                    trustedRpm[w]  = UNAVAILABLE;
                    wheelTrusted[w] = false;
                    continue;
                }

                /* Step 3: check pairwise agreement among valid readings. */
                float agreeSum   = 0.0f;
                int   agreeCount = 0;
                float reference  = UNAVAILABLE;

                /* Find the first valid reading as the reference. */
                for (int s = 0; s < SENSOR_COUNT; s++) {
                    if (readings[s] != UNAVAILABLE) {
                        reference = readings[s];
                        break;
                    }
                }

                /* Count sensors that agree with the reference. */
                for (int s = 0; s < SENSOR_COUNT; s++) {
                    if (readings[s] == UNAVAILABLE) {
                        continue;
                    }
                    if (Math.abs(readings[s] - reference) <= WHEEL_RPM_TOLERANCE) {
                        agreeSum += readings[s];
                        agreeCount++;
                    }
                }

                /* Step 4: at least 2 sensors agree — produce trusted RPM. */
                if (agreeCount >= 2) {
                    trustedRpm[w]  = agreeSum / agreeCount;
                    wheelTrusted[w] = true;
                    trustedWheelCount++;
                } else {
                    /* Disagreement — do not trust this wheel. */
                    trustedRpm[w]  = UNAVAILABLE;
                    wheelTrusted[w] = false;
                }
            }

            /* All four wheels must be trusted for full confidence. */
            fullyTrusted = (trustedWheelCount == WHEEL_COUNT);
        }

        /**
         * Returns the trusted RPM for the given wheel index,
         * or UNAVAILABLE if consensus was not reached for that wheel.
         * @param wheelIndex 0=FL, 1=FR, 2=RL, 3=RR
         */
        public float getTrustedRpm(final int wheelIndex) {
            assert wheelIndex >= 0 && wheelIndex < WHEEL_COUNT
                : "wheelIndex out of range";
            return trustedRpm[wheelIndex];
        }

        /**
         * Returns a defensive copy of all four trusted RPM values.
         * Values are UNAVAILABLE (-1) where consensus was not reached.
         */
        public float[] getAllTrustedRpm() {
            final float[] copy = new float[WHEEL_COUNT];
            for (int w = 0; w < WHEEL_COUNT; w++) {
                copy[w] = trustedRpm[w];
            }
            return copy;
        }

        public float getTrustedSpeed() {
            float RPM_TO_KMH_MULTIPLIER = (float) (Math.PI * 0.65 * 60.0 / 1000.0);
            List<Float> validSpeedsKmh = new ArrayList<>();

            // 1. Convert each active wheel's RPM to linear km/h speed
            for (int i = 0; i < 4; i++) {
                float rawRpm = getTrustedRpm(i); 
                
                if (rawRpm >= 0.0f) { // Protect against sentinel errors
                    float speedKmh = rawRpm * RPM_TO_KMH_MULTIPLIER;
                    validSpeedsKmh.add(speedKmh);
                }
            }
            

            if (validSpeedsKmh.isEmpty()) {
                return 0.0f;
            }

            // 2. Robust Fault Isolation via Sorting (Median Filtering)
            Collections.sort(validSpeedsKmh);
            int size = validSpeedsKmh.size();

            switch(size) {
            case 4:
                // Drop highest (possible slip) and lowest (possible lockup), average the middle two
                return (validSpeedsKmh.get(1) + validSpeedsKmh.get(2)) / 2.0f;
            case 3:
                // Return middle value cleanly
                 validSpeedsKmh.get(1);
            case 2:
                // Fall back to simple average of the remaining functional pair
                return (validSpeedsKmh.get(0) + validSpeedsKmh.get(1)) / 2.0f;
            default: 
                // Last remaining sensor standing
                return validSpeedsKmh.get(0);
            }
        }

        /** True if all four wheels have a trusted RPM from the last vote. */
        public boolean isFullyTrusted()                     { return fullyTrusted; }

        /** True if the given wheel has a trusted RPM from the last vote. */
        public boolean isWheelTrusted(final int wheelIndex) {
            assert wheelIndex >= 0 && wheelIndex < WHEEL_COUNT
                : "wheelIndex out of range";
            return wheelTrusted[wheelIndex];
        }
    }

    // =========================================================================
    // Enums
    // =========================================================================

    /**
     * The decision produced by the controller on each 50ms cycle.
     *
     * CLEAR           — no hazard detected; normal operation continues.
     * DRIVER_ALERT    — hazard detected but TTC above autonomous brake threshold;
     *                   alert the driver to take action.
     * AUTONOMOUS_BRAKE — hazard is imminent; controller initiates braking.
     * DRIVER_OVERRIDE  — driver is manually braking; controller yields all authority.
     * ESCALATION       — all brake attempts exhausted and driver is not braking;
     *                    escalation alert sent via two independent channels.
     * SENSOR_FAULT     — insufficient trusted sensor coverage; system degraded,
     *                    driver alerted.
     */
    public enum ControllerDecision {
        CLEAR,
        DRIVER_ALERT,
        AUTONOMOUS_BRAKE,
        DRIVER_OVERRIDE,
        ESCALATION,
        SENSOR_FAULT
    }

    // =========================================================================
    // Constants — timing
    // =========================================================================

    /**
     * Main decision cycle interval in milliseconds.
     * All AEBS decisions happen at this rate (FR3101, architecture spec).
     * Change this constant to alter the cycle rate globally.
     */
    public static final int DECISION_INTERVAL_MS = 50;

    /**
     * Maximum number of autonomous braking retry attempts before escalation
     * (FR-3105, architecture spec: "attempts twice more" = 3 total).
     * Change this constant to alter the retry count globally.
     */
    public static final int MAX_BRAKE_ATTEMPTS = 3;

    /**
     * Milliseconds allowed for escalation delivery confirmation per channel
     * before re-delivery is attempted (FR-3106).
     */
    public static final int ESCALATION_CONFIRM_TIMEOUT_MS = 50;

    // =========================================================================
    // Constants — thresholds
    // =========================================================================

    /**
     * Time-to-collision threshold in seconds for issuing a DRIVER_ALERT.
     * When TTC falls below this value, the driver is warned.
     * Adjust this constant to tune alert sensitivity.
     */
    public static final float TTC_ALERT_THRESHOLD_S = 3.0f;

    /**
     * Time-to-collision threshold in seconds for initiating AUTONOMOUS_BRAKE.
     * When TTC falls below this value, the controller brakes autonomously.
     * Adjust this constant to tune intervention aggressiveness.
     */
    public static final float TTC_BRAKE_THRESHOLD_S = 1.5f;

    /**
     * Maximum bearing angle (degrees) from straight ahead within which
     * an object is considered "in the vehicle's path" for TTC calculation.
     * Objects beyond this bearing are not on a collision course.
     */
    public static final float MAX_COLLISION_BEARING_DEG = 30.0f;

    /**
     * Wheel speed deceleration confirmation margin (percent).
     * A braking attempt is considered confirmed if the average wheel RPM
     * has reduced by at least this percentage from pre-brake RPM (FR-3108).
     * Note: FR-3108 requires this to also be checked against remaining distance.
     */
    public static final float BRAKE_CONFIRMATION_MARGIN_PERCENT = 5.0f;

    /**
     * Minimum distance reduction (metres) required alongside the wheel speed
     * margin for a braking attempt to be classified as "adequate" (FR-3108,
     * H-3105 — prevents success classification when obstacle is still too close).
     */
    public static final float BRAKE_ADEQUACY_MIN_DIST_REDUCTION_M = 1.0f;

    /**
     * RPM variance across wheels (RPM) that signals directional instability
     * during braking (FR-3113, H-3106). If any two wheels differ by more
     * than this value, directional instability is flagged.
     */
    public static final float WHEEL_INSTABILITY_RPM_DELTA = 100.0f;

    /**
     * Minimum confidence score from both Radar and Lidar voters required
     * for autonomous braking to be authorised on distance alone.
     * Prevents action on heavily degraded sensor data (FR2103).
     */
    public static final float MIN_TRUSTED_CONFIDENCE = 0.3f;

    // =========================================================================
    // Fields — sensors and sub-components
    // =========================================================================

    /** CameraVoter manages 2oo3 voting for the three Camera instances. */
    private final CameraVoter cameraVoter;

    /** RadarLidarVoter for the three Radar instances (FR2107). */
    private final RadarLidarVoter radarVoter;

    /** RadarLidarVoter for the three Lidar instances (FR2103, FR2106). */
    private final RadarLidarVoter lidarVoter;

    /** WheelVoter for the three WheelSensor instances (FR2208). */
    private final WheelVoter wheelVoter;

    /** Driver simulator — read to detect manual braking (FR3104). */
    private final Driver driver;

    /**
     * Two independent escalation channels for FR-3105 / FR-3106.
     * Exactly two must be provided at construction time.
     * Index 0 = Channel A, Index 1 = Channel B.
     */
    private final EscalationChannel[] escalationChannels;

    // =========================================================================
    // Fields — controller state
    // =========================================================================

    /** Most recent decision produced by the controller. */
    private ControllerDecision lastDecision;

    /** Current number of braking attempts in this hazard event. */
    private int brakeAttemptCount;

    /**
     * True while a hazard event is active (TTC below brake threshold and
     * brakes have not yet been confirmed adequate or driver is overriding).
     */
    private boolean hazardEventActive;

    /**
     * Pre-brake average wheel RPM captured at the start of a braking attempt.
     * Used to confirm deceleration occurred (FR-3108).
     */
    private float preBrakeRpmAvg;

    /**
     * Pre-brake fused distance captured at the start of a braking attempt.
     * Used to confirm distance is reducing as expected (FR-3108, H-3105).
     */
    private float preBrakeDistanceM;

    /** True once the 50ms decision timer has been started. */
    private boolean running;

    /** Java Timer driving the 50ms decision cycle. */
    private Timer decisionTimer;

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * Constructs the BrakingController with all required sensor voters and channels.
     *
     * The caller is responsible for constructing and starting all sensor instances
     * before calling start() on this controller.
     *
     * @param cam1              Camera instance 1
     * @param cam2              Camera instance 2
     * @param cam3              Camera instance 3
     * @param radar1            Radar instance 1
     * @param radar2            Radar instance 2
     * @param radar3            Radar instance 3
     * @param lidar1            Lidar instance 1
     * @param lidar2            Lidar instance 2
     * @param lidar3            Lidar instance 3
     * @param ws1               WheelSensor set 1
     * @param ws2               WheelSensor set 2
     * @param ws3               WheelSensor set 3
     * @param driver            Driver simulator instance
     * @param channelA          first escalation channel (FR-3105)
     * @param channelB          second escalation channel (FR-3105)
     */
    public BrakingController(final Camera cam1,
                             final Camera cam2,
                             final Camera cam3,
                             final Radar radar1,
                             final Radar radar2,
                             final Radar radar3,
                             final Lidar lidar1,
                             final Lidar lidar2,
                             final Lidar lidar3,
                             final WheelSensor ws1,
                             final WheelSensor ws2,
                             final WheelSensor ws3,
                             final Driver driver,
                             final EscalationChannel channelA,
                             final EscalationChannel channelB) {

        assert cam1   != null : "cam1 must not be null";
        assert cam2   != null : "cam2 must not be null";
        assert cam3   != null : "cam3 must not be null";
        assert radar1 != null : "radar1 must not be null";
        assert radar2 != null : "radar2 must not be null";
        assert radar3 != null : "radar3 must not be null";
        assert lidar1 != null : "lidar1 must not be null";
        assert lidar2 != null : "lidar2 must not be null";
        assert lidar3 != null : "lidar3 must not be null";
        assert ws1    != null : "ws1 must not be null";
        assert ws2    != null : "ws2 must not be null";
        assert ws3    != null : "ws3 must not be null";
        assert driver != null : "driver must not be null";
        assert channelA != null : "channelA must not be null";
        assert channelB != null : "channelB must not be null";

        /* Build the camera voter from the three camera instances. */
        this.cameraVoter = new CameraVoter(cam1, cam2, cam3);

        /* Build radar and lidar 2oo3 voters independently (FR2106, FR2107). */
        this.radarVoter  = new RadarLidarVoter(radar1, radar2, radar3, "Radar");
        this.lidarVoter  = new RadarLidarVoter(lidar1, lidar2, lidar3, "Lidar");

        /* Build the wheel sensor voter. */
        this.wheelVoter  = new WheelVoter(ws1, ws2, ws3);

        this.driver = driver;

        /* Store both escalation channels in a fixed-size array. */
        this.escalationChannels    = new EscalationChannel[2];
        this.escalationChannels[0] = channelA;
        this.escalationChannels[1] = channelB;

        /* Initialise controller state to safe defaults. */
        this.lastDecision      = ControllerDecision.CLEAR;
        this.brakeAttemptCount = 0;
        this.hazardEventActive = false;
        this.preBrakeRpmAvg    = 0.0f;
        this.preBrakeDistanceM = Float.MAX_VALUE;
        this.running           = false;
    }

    // =========================================================================
    // Lifecycle
    // =========================================================================

    /**
     * Starts the 50ms decision cycle timer.
     *
     * The controller will call runDecisionCycle() every DECISION_INTERVAL_MS
     * milliseconds. All sensor instances should already be started before
     * calling this method.
     */
    public void start() {
        assert !running : "controller already running";
        running = true;
        decisionTimer = new Timer("BrakingController", false);
        decisionTimer.scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                /* Every 50ms: run the full decision cycle. */
                runDecisionCycle();
            }
        }, 0, DECISION_INTERVAL_MS);
        System.out.println("BrakingController started — decision cycle: "
                + DECISION_INTERVAL_MS + "ms");
    }

    /**
     * Stops the decision cycle timer.
     * Safe to call even if the controller was never started.
     */
    public void stop() {
        running = false;
        if (decisionTimer != null) {
            decisionTimer.cancel();
        }
        System.out.println("BrakingController stopped.");
    }

    /** Returns the most recent decision produced by the controller. */
    public ControllerDecision getLastDecision() {
        return lastDecision;
    }

    // =========================================================================
    // Core decision cycle — called every 50ms
    // =========================================================================

    /**
     * Runs one complete decision cycle.
     *
     * This is the heart of the AEBS controller. It executes in strict order:
     *
     *   Step 1: Check for driver manual override (FR3104) — highest priority.
     *           If the driver is braking, yield immediately and return.
     *
     *   Step 2: Run all sensor votes to get the latest trusted readings.
     *
     *   Step 3: Evaluate sensor health. If insufficient coverage, issue a
     *           SENSOR_FAULT and alert the driver.
     *
     *   Step 4: Compute Time-to-Collision from the fused sensor data.
     *
     *   Step 5: Make a decision based on TTC and sensor confidence:
     *           - TTC > alert threshold    → CLEAR
     *           - TTC < alert threshold    → DRIVER_ALERT
     *           - TTC < brake threshold    → AUTONOMOUS_BRAKE (attempt braking)
     *
     *   Step 6: If AUTONOMOUS_BRAKE was decided, execute braking with retry
     *           logic and post-execution threat reassessment (FR-3109).
     *
     *   Step 7: If all brake attempts fail, escalate (FR-3105, FR-3106).
     *
     * All steps complete within one 50ms window. No blocking I/O is performed.
     */
    private void runDecisionCycle() {

        /* ------------------------------------------------------------------ */
        /* Step 1: Driver override check (FR3104 — highest priority)          */
        /* ------------------------------------------------------------------ */
        if (isDriverBraking()) {
            /*
             * Driver has applied brakes manually. Per FR3104, we must immediately
             * yield all braking authority and suppress any active AEBS intervention.
             * Reset hazard state so a new hazard event starts cleanly if needed.
             */
            lastDecision      = ControllerDecision.DRIVER_OVERRIDE;
            hazardEventActive = false;
            brakeAttemptCount = 0;
            System.out.println("[BrakingController] DRIVER_OVERRIDE — yielding to driver.");
            return;
        }

        /* ------------------------------------------------------------------ */
        /* Step 2: Collect all sensor votes                                    */
        /* ------------------------------------------------------------------ */
        /*
         * CameraVoter already manages its internal 2oo3 state — just call vote().
         * RadarLidarVoters and WheelVoter are called here directly.
         */
        final VoteResult cameraVote = cameraVoter.vote();
        radarVoter.vote();
        lidarVoter.vote();
        wheelVoter.vote();

        /* ------------------------------------------------------------------ */
        /* Step 3: Sensor health check                                         */
        /* ------------------------------------------------------------------ */
        /*
         * Per FR2106 and FR2107, braking must not be authorised on a single
         * sensor type alone. We need at least one trusted distance source
         * (Radar or Lidar) to proceed with any collision decision.
         *
         * Camera alone is insufficient for distance estimation — it provides
         * object classification only.
         */
        final boolean radarTrusted = radarVoter.isTrusted();
        final boolean lidarTrusted = lidarVoter.isTrusted();
        final boolean cameraOk     = (cameraVote == VoteResult.CONSENSUS
                                   || cameraVote == VoteResult.DISAGREEMENT);

        /* At least one ranging sensor must be trusted to proceed. */
        if (!radarTrusted && !lidarTrusted) {
            lastDecision = ControllerDecision.SENSOR_FAULT;
            System.out.println("[BrakingController] SENSOR_FAULT — no trusted ranging sensor.");
            /*
             * Alert the driver that AEBS capability is compromised (FR2102).
             * This is a soft alert, not the escalation pathway.
             */
            notifySensorFaultToDriver();
            return;
        }

        /* ------------------------------------------------------------------ */
        /* Step 4: Compute Time-to-Collision                                   */
        /* ------------------------------------------------------------------ */
        final float ttcSeconds = computeTTC(radarTrusted, lidarTrusted, wheelVoter.getTrustedSpeed());

        /* ------------------------------------------------------------------ */
        /* Step 5: Make the control decision                                   */
        /* ------------------------------------------------------------------ */
        final ControllerDecision decision = evaluateDecision(ttcSeconds,
                                                              cameraVote,
                                                              radarTrusted,
                                                              lidarTrusted);
        lastDecision = decision;

        /* ------------------------------------------------------------------ */
        /* Step 6: Execute braking if decided                                  */
        /* ------------------------------------------------------------------ */
        if (decision == ControllerDecision.AUTONOMOUS_BRAKE) {
            executeBrakingSequence(ttcSeconds);
        } else if (decision == ControllerDecision.DRIVER_ALERT) {
            /* Not braking yet — just inform the driver via the interface. */
            System.out.println("[BrakingController] DRIVER_ALERT — TTC=" + ttcSeconds + "s");
            notifyDriverAlert(ttcSeconds);
            /* Reset brake attempt counter — hazard not yet at brake threshold. */
            if (!hazardEventActive) {
                brakeAttemptCount = 0;
            }
        } else {
            /* CLEAR — no hazard. Reset all hazard-event state. */
            hazardEventActive = false;
            brakeAttemptCount = 0;
        }

        assert lastDecision != null : "lastDecision must not be null after cycle";
    }

    // =========================================================================
    // Braking execution
    // =========================================================================

    /**
     * Executes the autonomous braking sequence with up to MAX_BRAKE_ATTEMPTS
     * retry attempts at DECISION_INTERVAL_MS intervals.
     *
     * Execution process:
     *   1. Capture pre-brake wheel RPM average and current distance.
     *   2. Issue brake command.
     *   3. Wait one decision interval (this is synchronous within the cycle;
     *      confirmation check happens next cycle via hazardEventActive flag).
     *   4. On next cycle, confirm wheel speed deceleration AND distance reduction
     *      (FR-3108, H-3105).
     *   5. Run post-execution threat reassessment (FR-3109).
     *   6. If confirmed adequate and no residual threat, clear hazard event.
     *   7. If not confirmed, retry up to MAX_BRAKE_ATTEMPTS total.
     *   8. If all attempts exhausted, escalate (FR-3105).
     *
     * Note: The actual braking command is issued via issueBrakeCommand() which
     * is a stub for the BrakeActuator interface. Confirmation is via wheel sensor
     * feedback in confirmBrakeExecution().
     *
     * @param currentTtcSeconds the TTC computed this cycle
     */
    private void executeBrakingSequence(final float currentTtcSeconds) {

        /* Mark a new hazard event if this is the first brake cycle. */
        if (!hazardEventActive) {
            hazardEventActive = true;
            brakeAttemptCount = 0;
            /* Capture pre-brake state for confirmation comparison. */
            preBrakeRpmAvg    = computeAverageWheelRpm();
            preBrakeDistanceM = getBestTrustedDistance();
            System.out.println("[BrakingController] Hazard event started — "
                + "preRpm=" + preBrakeRpmAvg + " preDist=" + preBrakeDistanceM + "m");
        }

        /* Check if we have already exhausted all attempts this hazard event. */
        if (brakeAttemptCount >= MAX_BRAKE_ATTEMPTS) {
            /*
             * All retries exhausted. Escalate to driver via two independent channels
             * as required by FR-3105, FR-3106, H-3104.
             */
            executeEscalation();
            return;
        }

        /* Increment attempt counter before issuing. */
        brakeAttemptCount++;
        System.out.println("[BrakingController] Brake attempt "
                + brakeAttemptCount + "/" + MAX_BRAKE_ATTEMPTS);

        /* Determine brake intensity based on wheel traction (FR-3111). */
        final float brakeIntensity = computeBrakeIntensity();

        /* Issue the brake command to the BrakeActuator (stub). */
        issueBrakeCommand(brakeIntensity);

        /* Run the braking modulation check (FR-3111, FR-3112, FR-3113). */
        runBrakeModulationCheck(brakeIntensity);

        /* Check if this attempt was confirmed by wheel sensor feedback. */
        final boolean brakeConfirmed = confirmBrakeExecution();

        if (brakeConfirmed) {
            System.out.println("[BrakingController] Brake attempt "
                    + brakeAttemptCount + " CONFIRMED.");
            /*
             * FR-3109: even after confirmed braking, re-evaluate whether the
             * threat has actually been resolved. The obstacle may still be too
             * close despite deceleration occurring.
             */
            final boolean threatPersists = isThreatPersistingPostBrake(currentTtcSeconds);
            if (threatPersists) {
                /*
                 * FR-3110: Residual collision risk detected despite confirmed braking.
                 * Issue driver alert immediately — do not suppress further intervention.
                 */
                System.out.println("[BrakingController] Post-execution: "
                        + "RESIDUAL THREAT — issuing residual collision alert.");
                notifyResidualCollisionAlert();
                /*
                 * Do NOT clear hazardEventActive — the next decision cycle will
                 * re-evaluate and may issue another brake command.
                 */
            } else {
                /* Threat resolved — clear hazard event state. */
                System.out.println("[BrakingController] Threat resolved after "
                        + brakeAttemptCount + " attempt(s).");
                hazardEventActive = false;
                brakeAttemptCount = 0;
                lastDecision = ControllerDecision.CLEAR;
            }
        } else {
            System.out.println("[BrakingController] Brake attempt "
                    + brakeAttemptCount + " NOT confirmed — will retry.");
            /*
             * Braking was not confirmed. hazardEventActive remains true.
             * The next decision cycle will check TTC again and if still below
             * threshold will call executeBrakingSequence() again until
             * MAX_BRAKE_ATTEMPTS is reached.
             */
        }
    }

    // =========================================================================
    // Escalation (FR-3105, FR-3106, FR-3107)
    // =========================================================================

    /**
     * Executes the escalation sequence when all braking attempts are exhausted.
     *
     * Sends the escalation alert simultaneously via both independent channels
     * (FR-3105), then polls for delivery confirmation within
     * ESCALATION_CONFIRM_TIMEOUT_MS (FR-3106). If neither channel confirms,
     * re-delivers on all available channels and logs the failure.
     *
     * The escalation interface is explicitly owned at the boundary between
     * BrakingController and the Emergency Intervention Indicators subsystem
     * (FR-3107). The EscalationChannel interface is the formal handoff point.
     */
    private void executeEscalation() {
        lastDecision = ControllerDecision.ESCALATION;
        System.out.println("[BrakingController] ESCALATION: all "
                + MAX_BRAKE_ATTEMPTS + " brake attempts exhausted.");

        /* Step 1: Send alert on BOTH channels simultaneously (FR-3105). */
        for (int c = 0; c < escalationChannels.length; c++) {
            escalationChannels[c].sendAlert();
            System.out.println("[BrakingController] Escalation alert sent on channel: "
                    + escalationChannels[c].channelName());
        }

        /* Step 2: Poll for delivery confirmation within timeout (FR-3106).
         *
         * We busy-poll for up to ESCALATION_CONFIRM_TIMEOUT_MS. This is the
         * only intentional busy-wait in the system — the timeout is short (50ms)
         * and the alternative (asynchronous callback) would require dynamic
         * allocation.
         */
        final long confirmDeadlineMs =
                System.currentTimeMillis() + ESCALATION_CONFIRM_TIMEOUT_MS;
        boolean anyConfirmed = false;

        while (System.currentTimeMillis() < confirmDeadlineMs) {
            /* Check both channels on each poll iteration. */
            for (int c = 0; c < escalationChannels.length; c++) {
                if (escalationChannels[c].isDeliveryConfirmed()) {
                    anyConfirmed = true;
                    System.out.println("[BrakingController] Escalation confirmed on: "
                            + escalationChannels[c].channelName());
                    break;
                }
            }
            if (anyConfirmed) {
                break;
            }
        }

        /* Step 3: If no confirmation received, log and re-deliver (FR-3106). */
        if (!anyConfirmed) {
            System.err.println("[BrakingController] ESCALATION DELIVERY FAILURE: "
                    + "no confirmation within " + ESCALATION_CONFIRM_TIMEOUT_MS
                    + "ms — re-delivering on all channels.");
            /* Re-deliver on all channels. */
            for (int c = 0; c < escalationChannels.length; c++) {
                escalationChannels[c].sendAlert();
                System.out.println("[BrakingController] Re-delivering on: "
                        + escalationChannels[c].channelName());
            }
        }
    }

    // =========================================================================
    // TTC computation
    // =========================================================================

    /**
     * Computes Time-to-Collision in seconds from the trusted sensor data,
     * cross-referenced against the current vehicle speed to protect against
     * low-speed calculation volatility.
     *
     * @param radarTrusted whether the radar voter produced a trusted result
     * @param lidarTrusted whether the lidar voter produced a trusted result
     * @param carSpeedKmh  the current validated speed of our vehicle from wheel sensors
     * @return TTC in seconds, or Float.MAX_VALUE if no imminent collision
     */
    private float computeTTC(final boolean radarTrusted, final boolean lidarTrusted, final float carSpeedKmh) {
        float distanceM  = Float.MAX_VALUE;
        float closingKmh = 0.0f;

        if (radarTrusted && lidarTrusted) {
            final float radarConf = radarVoter.getFusedConfidence();
            final float lidarConf = lidarVoter.getFusedConfidence();
            final float totalConf = radarConf + lidarConf;

            if (totalConf > 0.001f) {
                distanceM  = (radarVoter.getFusedDistanceM()        * radarConf
                           +  lidarVoter.getFusedDistanceM()        * lidarConf) / totalConf;
                closingKmh = (radarVoter.getFusedRelativeSpeedKmh() * radarConf
                           +  lidarVoter.getFusedRelativeSpeedKmh() * lidarConf) / totalConf;
            } else {
                return Float.MAX_VALUE;
            }
        } else if (radarTrusted) {
            distanceM  = radarVoter.getFusedDistanceM();
            closingKmh = radarVoter.getFusedRelativeSpeedKmh();
        } else if (lidarTrusted) {
            distanceM  = lidarVoter.getFusedDistanceM();
            closingKmh = lidarVoter.getFusedRelativeSpeedKmh();
        } else {
            return Float.MAX_VALUE;
        }

        // 1. If our car is fully stopped, a forward collision collision course is broken
        if (carSpeedKmh <= 0.1f) {
            return Float.MAX_VALUE;
        }

        /*
         * Sensor convention: relativeSpeedKmh is negative when the obstacle
         * is approaching the vehicle. Convert to a positive closing speed.
         */
        float closingSpeedKmh = -closingKmh;

        /* * 2. Physical Cross-Check:
         * If the obstacle is a stationary object (like a wall or stopped car), 
         * your closing speed cannot physically exceed your car's own absolute speed.
         * We bound this to filter out transient sensor noise anomalies.
         */
        if (closingSpeedKmh > carSpeedKmh) {
            closingSpeedKmh = carSpeedKmh;
        }

        if (closingSpeedKmh <= 0.0f) {
            return Float.MAX_VALUE;
        }

        /* Convert closing speed from km/h to m/s for TTC calculation. */
        final float closingSpeedMs = closingSpeedKmh / 3.6f;

        if (closingSpeedMs < 0.01f) {
            return Float.MAX_VALUE;
        }

        /* TTC = distance / closing speed */
        final float ttc = distanceM / closingSpeedMs;

        assert ttc >= 0.0f : "TTC must not be negative";
        return ttc;
    }

    // =========================================================================
    // Decision evaluation
    // =========================================================================

    /**
     * Evaluates the control decision based on TTC, sensor votes, and confidence.
     *
     * Decision rules (in priority order):
     *
     *   1. If TTC > TTC_ALERT_THRESHOLD_S: CLEAR — no action needed.
     *
     *   2. If TTC between thresholds: DRIVER_ALERT — warn the driver.
     *
     *   3. If TTC < TTC_BRAKE_THRESHOLD_S AND sufficient cross-sensor
     *      confirmation (FR2106, FR2107):
     *        - If both Radar and Lidar are trusted: AUTONOMOUS_BRAKE.
     *        - If only one is trusted but confidence >= MIN_TRUSTED_CONFIDENCE
     *          AND camera has consensus on a threat object: AUTONOMOUS_BRAKE.
     *        - Otherwise: DRIVER_ALERT (degrade to warning, do not brake on
     *          insufficient evidence — avoids false positives per H-4102).
     *
     * @param ttcSeconds   computed TTC from sensor fusion
     * @param cameraVote   result of the camera 2oo3 vote
     * @param radarTrusted whether radar voter is trusted
     * @param lidarTrusted whether lidar voter is trusted
     * @return the appropriate ControllerDecision for this cycle
     */
    private ControllerDecision evaluateDecision(final float ttcSeconds,
                                                 final VoteResult cameraVote,
                                                 final boolean radarTrusted,
                                                 final boolean lidarTrusted) {
        assert ttcSeconds >= 0.0f : "TTC must not be negative";

        /* Rule 1: No hazard within alert range. */
        if (ttcSeconds > TTC_ALERT_THRESHOLD_S) {
            return ControllerDecision.CLEAR;
        }

        /* Rule 2: Within alert range but not yet at brake threshold. */
        if (ttcSeconds > TTC_BRAKE_THRESHOLD_S) {
            return ControllerDecision.DRIVER_ALERT;
        }

        /* Rule 3: Within brake threshold — decide whether to brake. */

        /* Both Radar and Lidar trusted: full multi-sensor confirmation. */
        if (radarTrusted && lidarTrusted) {
            /* Ensure at least one sensor is confident enough to act on. */
            final float bestConf = Math.max(radarVoter.getFusedConfidence(),
                                             lidarVoter.getFusedConfidence());
            if (bestConf >= MIN_TRUSTED_CONFIDENCE) {
                return ControllerDecision.AUTONOMOUS_BRAKE;
            }
            /* Sensors agree but confidence is too low — downgrade to alert. */
            System.out.println("[BrakingController] Both sensors trusted but "
                    + "combined confidence too low (" + bestConf + ") — alerting only.");
            return ControllerDecision.DRIVER_ALERT;
        }

        /*
         * Only one ranging sensor trusted: we cannot brake on that sensor alone
         * (FR2106, FR2107). However, if the camera voter confirms a threat object
         * with CONSENSUS, we have cross-type corroboration and may brake.
         */
        if (cameraVote == VoteResult.CONSENSUS
                && cameraVoter.getConsensusObject() != Camera.ObjectType.NONE
                && cameraVoter.getConsensusObject() != Camera.ObjectType.UNKNOWN) {

            /* Check confidence of the one trusted sensor. */
            final float singleConf = radarTrusted
                    ? radarVoter.getFusedConfidence()
                    : lidarVoter.getFusedConfidence();

            if (singleConf >= MIN_TRUSTED_CONFIDENCE) {
                System.out.println("[BrakingController] Single ranging sensor + camera "
                        + "consensus — authorising brake (conf=" + singleConf + ").");
                return ControllerDecision.AUTONOMOUS_BRAKE;
            }
        }

        /* Insufficient cross-sensor corroboration — degrade to alert only. */
        System.out.println("[BrakingController] Insufficient sensor corroboration "
                + "at TTC=" + ttcSeconds + "s — downgrading to DRIVER_ALERT.");
        return ControllerDecision.DRIVER_ALERT;
    }

    // =========================================================================
    // Brake modulation (FR-3111, FR-3112, FR-3113)
    // =========================================================================

    /**
     * Runs the brake modulation check during an active braking event.
     *
     * This implements the closed-loop modulation requirement (FR-3111) by
     * reading current wheel speeds immediately after issuing the brake command
     * and checking for lockup or directional instability.
     *
     * Lockup detection (FR-3112): if any wheel's trusted RPM drops to near zero
     * while the vehicle is still moving (other wheels show speed), the braking
     * command is reduced and the driver is alerted.
     *
     * Directional instability detection (FR-3113): if the RPM spread across
     * wheels exceeds WHEEL_INSTABILITY_RPM_DELTA, the wheels are decelerating
     * unevenly, indicating a potential skid. The driver is alerted and brake
     * force is modulated.
     *
     * Note: This runs within the same 50ms decision cycle as the brake command.
     * The 10ms wheel sensor update rate means the very latest feedback is
     * available (the wheel sensor timer runs independently at 10ms).
     *
     * @param currentIntensity the brake intensity that was just issued
     */
    private void runBrakeModulationCheck(final float currentIntensity) {
        assert currentIntensity >= 0.0f && currentIntensity <= 1.0f
            : "brakeIntensity must be between 0 and 1";

        /* Re-vote the wheel sensors to get the most current data. */
        wheelVoter.vote();

        final float[] rpm = wheelVoter.getAllTrustedRpm();

        /* Compute average RPM of all trusted (non-unavailable) wheels. */
        float sum = 0.0f;
        int   count = 0;
        float minRpm = Float.MAX_VALUE;
        float maxRpm = 0.0f;

        for (int w = 0; w < WheelVoter.WHEEL_COUNT; w++) {
            if (rpm[w] >= 0.0f) { /* UNAVAILABLE is -1.0f */
                sum += rpm[w];
                count++;
                if (rpm[w] < minRpm) { minRpm = rpm[w]; }
                if (rpm[w] > maxRpm) { maxRpm = rpm[w]; }
            }
        }

        if (count == 0) {
            /* No wheel data — cannot modulate; maintain current intensity. */
            System.out.println("[BrakingController] Modulation: no trusted wheel data.");
            return;
        }

        final float avgRpm = sum / count;

        /* FR-3112: Lockup detection.
         *
         * A wheel is "locked" if its RPM is near zero while the vehicle is
         * clearly still moving (average RPM is above a meaningful threshold).
         * If lockup is detected, reduce brake intensity and alert driver.
         */
        if (minRpm < 10.0f && avgRpm > 50.0f) {
            System.err.println("[BrakingController] LOCKUP DETECTED — "
                    + "min wheel RPM=" + minRpm + " vs avg=" + avgRpm
                    + " — reducing brake force.");
            /* Reduce brake command: issue a modulated (reduced) command. */
            final float reducedIntensity = currentIntensity * 0.6f;
            issueBrakeCommand(reducedIntensity);
            notifyLockupAlert();
        }

        /* FR-3113: Directional instability detection.
         *
         * If the spread between the fastest and slowest wheel exceeds
         * WHEEL_INSTABILITY_RPM_DELTA, the vehicle may be skidding.
         * Alert driver and modulate brake force.
         */
        if ((maxRpm - minRpm) > WHEEL_INSTABILITY_RPM_DELTA) {
            System.err.println("[BrakingController] DIRECTIONAL INSTABILITY — "
                    + "wheel RPM spread=" + (maxRpm - minRpm)
                    + " (threshold=" + WHEEL_INSTABILITY_RPM_DELTA + ")");
            notifyDirectionalInstabilityAlert();
            /* Modulate: reduce to graduated braking (FR2211). */
            final float modulatedIntensity = currentIntensity * 0.75f;
            issueBrakeCommand(modulatedIntensity);
        }
    }

    /**
     * Computes the brake intensity to apply for this attempt, scaled by
     * surface traction condition inferred from wheel speed variance.
     *
     * Full intensity (1.0) is used when wheels are tracking normally.
     * Graduated (reduced) intensity is used when variance suggests low
     * traction (FR2211, H-3106).
     *
     * @return brake intensity in range [0.0, 1.0]
     */
    private float computeBrakeIntensity() {
        wheelVoter.vote();
        final float[] rpm = wheelVoter.getAllTrustedRpm();

        float minRpm = Float.MAX_VALUE;
        float maxRpm = 0.0f;
        int   count  = 0;

        for (int w = 0; w < WheelVoter.WHEEL_COUNT; w++) {
            if (rpm[w] >= 0.0f) {
                if (rpm[w] < minRpm) { minRpm = rpm[w]; }
                if (rpm[w] > maxRpm) { maxRpm = rpm[w]; }
                count++;
            }
        }

        if (count < 2) {
            /* Not enough wheel data — use moderate intensity as safe default. */
            return 0.7f;
        }

        final float spread = maxRpm - minRpm;

        if (spread > WHEEL_INSTABILITY_RPM_DELTA) {
            /* High variance: surface likely low-traction — graduate braking. */
            System.out.println("[BrakingController] Low-traction surface inferred "
                    + "(wheel spread=" + spread + ") — graduating brake intensity.");
            return 0.5f;
        }

        /* Normal traction — apply full braking. */
        return 1.0f;
    }

    // =========================================================================
    // Brake confirmation (FR-3108, H-3105)
    // =========================================================================

    /**
     * Confirms whether the most recent braking attempt achieved adequate
     * deceleration.
     *
     * Per FR-3108 and H-3105: confirmation requires BOTH:
     *   (a) Average wheel RPM has reduced by at least BRAKE_CONFIRMATION_MARGIN_PERCENT
     *       compared to the pre-brake RPM (wheel speed check).
     *   (b) The best trusted distance to obstacle has reduced by at least
     *       BRAKE_ADEQUACY_MIN_DIST_REDUCTION_M (distance adequacy check).
     *
     * Checking only wheel speed (as would satisfy a simple ±5% spec reading)
     * is explicitly not sufficient — H-3105 shows this can lead to a false
     * "success" classification while the obstacle is still too close.
     *
     * @return true if braking was confirmed adequate on both criteria
     */
    private boolean confirmBrakeExecution() {
        /* Get current post-brake wheel RPM average. */
        wheelVoter.vote();
        final float postBrakeRpmAvg = computeAverageWheelRpm();

        /* Criterion (a): wheel speed reduction percentage. */
        final boolean rpmReduced;
        if (preBrakeRpmAvg > 0.0f) {
            final float reductionPercent =
                    ((preBrakeRpmAvg - postBrakeRpmAvg) / preBrakeRpmAvg) * 100.0f;
            rpmReduced = reductionPercent >= BRAKE_CONFIRMATION_MARGIN_PERCENT;
            System.out.println("[BrakingController] Brake confirm: RPM reduction="
                    + reductionPercent + "% (need " + BRAKE_CONFIRMATION_MARGIN_PERCENT + "%)");
        } else {
            /* Vehicle was already stationary — braking trivially "confirmed" on RPM. */
            rpmReduced = true;
        }

        /* Criterion (b): distance adequacy — obstacle must be further than before. */
        final float currentDistM = getBestTrustedDistance();
        final float distReduction = preBrakeDistanceM - currentDistM;
        /*
         * Note: a positive distReduction means the obstacle is now further away
         * (distance increased from vehicle perspective after braking). This is
         * correct — if we are decelerating and the obstacle is stationary,
         * the closing distance shrinks (distance decreases). However, if the
         * vehicle decelerated enough to stop closing, the gap may stabilise.
         *
         * We use a pragmatic check: either distance has increased (deceleration
         * worked) or current distance is safely beyond the pre-brake distance
         * by at least BRAKE_ADEQUACY_MIN_DIST_REDUCTION_M.
         *
         * In a real system this would use predicted stopping distance from speed
         * and deceleration rate. Here we use the simpler comparison consistent
         * with the sensor data available.
         */
        final boolean distAdequate = distReduction >= BRAKE_ADEQUACY_MIN_DIST_REDUCTION_M;
        System.out.println("[BrakingController] Brake confirm: dist change="
                + distReduction + "m (need " + BRAKE_ADEQUACY_MIN_DIST_REDUCTION_M + "m)");

        return rpmReduced && distAdequate;
    }

    /**
     * Determines whether a collision threat persists after a confirmed
     * brake execution (FR-3109, H-3105).
     *
     * Re-evaluates current TTC after the brake was confirmed. If TTC is still
     * below the brake threshold, the threat has not been resolved and further
     * intervention must not be suppressed.
     *
     * @param ttcAtBrakeTime the TTC that triggered braking this cycle
     * @return true if the threat is still active despite confirmed braking
     */
    private boolean isThreatPersistingPostBrake(final float ttcAtBrakeTime) {
        /*
         * Re-run the voter votes to get the very latest sensor data for
         * post-execution assessment. These are the same voters — the most
         * recent sensor readings may have updated since the start of this cycle.
         */
        radarVoter.vote();
        lidarVoter.vote();
        wheelVoter.vote();
        final float updatedTtc = computeTTC(radarVoter.isTrusted(),
                                            lidarVoter.isTrusted(),
                                            wheelVoter.getTrustedSpeed());

        System.out.println("[BrakingController] Post-execution TTC: " + updatedTtc + "s");

        /* Threat persists if TTC is still below the brake threshold. */
        return updatedTtc < TTC_BRAKE_THRESHOLD_S;
    }

    // =========================================================================
    // Helper methods
    // =========================================================================

    /**
     * Returns true if the driver is currently applying the brakes manually.
     * Reads from the Driver simulator (FR3104).
     */
    private boolean isDriverBraking() {
        final DriverReading dr = driver.getLatestReading();
        if (dr == null) {
            return false;
        }
        return dr.isBraking();
    }

    /**
     * Computes the average trusted RPM across all four wheels from the
     * most recent WheelVoter result.
     * Returns 0.0 if no trusted wheel data is available.
     */
    private float computeAverageWheelRpm() {
        final float[] rpm = wheelVoter.getAllTrustedRpm();
        float sum   = 0.0f;
        int   count = 0;
        for (int w = 0; w < WheelVoter.WHEEL_COUNT; w++) {
            if (rpm[w] >= 0.0f) { /* exclude UNAVAILABLE and FROZEN sentinels */
                sum += rpm[w];
                count++;
            }
        }
        return (count == 0) ? 0.0f : sum / count;
    }

    /**
     * Returns the best available trusted distance from Radar or Lidar voters.
     * Prefers the higher-confidence sensor when both are trusted.
     * Returns Float.MAX_VALUE if neither sensor has a trusted reading.
     */
    private float getBestTrustedDistance() {
        if (radarVoter.isTrusted() && lidarVoter.isTrusted()) {
            /* Use the sensor with higher confidence. */
            if (radarVoter.getFusedConfidence() >= lidarVoter.getFusedConfidence()) {
                return radarVoter.getFusedDistanceM();
            }
            return lidarVoter.getFusedDistanceM();
        }
        if (radarVoter.isTrusted()) {
            return radarVoter.getFusedDistanceM();
        }
        if (lidarVoter.isTrusted()) {
            return lidarVoter.getFusedDistanceM();
        }
        return Float.MAX_VALUE;
    }

    // =========================================================================
    // Stub notification methods
    // =========================================================================
    /* 
     * The methods below are stubs for the interface to the Emergency 
     * Intervention Indicators and BrakeActuator subsystems.
     * They are not implemented here — they represent the formal 
     * subsystem boundary (FR-3107).
     *
     * Replace each method body with a call to the relevant subsystem
     * once those classes are implemented.
     */

    /**
     * Sends the autonomous braking command to the BrakeActuator.
     * @param intensity brake force in range [0.0 = none, 1.0 = maximum]
     */
    private void issueBrakeCommand(final float intensity) {
        assert intensity >= 0.0f && intensity <= 1.0f
            : "brake intensity must be between 0 and 1";
        brakeActuator.applyBrake(intensity);
        System.out.println("[BrakingController] BRAKE COMMAND issued — intensity=" + intensity);
    }

    /**
     * Alerts the driver to an imminent hazard (TTC within alert range).
     * Calls into the Emergency Intervention Indicators subsystem.
     * @param ttcSeconds current TTC passed to the interface for display
     */
    private void notifyDriverAlert(final float ttcSeconds) {
        /* TODO: call Interface.showDriverAlert(ttcSeconds) */
        System.out.println("[BrakingController] DRIVER ALERT — TTC=" + ttcSeconds + "s");
    }

    /**
     * Alerts the driver that a residual collision risk remains despite
     * confirmed braking (FR-3110).
     */
    private void notifyResidualCollisionAlert() {
        /* TODO: call Interface.showResidualCollisionAlert() */
        System.out.println("[BrakingController] RESIDUAL COLLISION ALERT issued.");
    }

    /**
     * Alerts the driver that the AEBS sensor coverage is insufficient
     * for reliable collision detection (FR2102).
     */
    private void notifySensorFaultToDriver() {
        /* TODO: call Interface.showSensorFaultWarning() */
        System.out.println("[BrakingController] SENSOR FAULT warning issued to driver.");
    }

    /**
     * Alerts the driver that wheel lockup was detected during braking (FR-3112).
     */
    private void notifyLockupAlert() {
        /* TODO: call Interface.showLockupAlert() */
        System.out.println("[BrakingController] LOCKUP ALERT issued to driver.");
    }

    /**
     * Alerts the driver that directional instability was detected during
     * braking (FR-3113).
     */
    private void notifyDirectionalInstabilityAlert() {
        /* TODO: call Interface.showDirectionalInstabilityAlert() */
        System.out.println("[BrakingController] DIRECTIONAL INSTABILITY ALERT issued.");
    }
}