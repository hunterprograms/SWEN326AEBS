# AEBS Test Framework

Integration test suite for the Automatic Emergency Braking System (AEBS).
Covers fault-tolerance, sensor voting (2oo3), driver interaction, and escalation paths.

---

## Architecture overview

```
AEBSTestSuite          ← catalogue of all test cases (the only file you normally edit)
    │
    ▼
AEBSTestRunner         ← executes scenarios, collects decisions, evaluates assertions
    │
    ├── FaultInjector  ← thin wrappers: FaultyCamera, FaultyRadarLidar,
    │                     FaultyWheelSensor, FaultyDriver
    │
    ├── BrakingControllerTestAdapter  ← bridges RadarLidarSensor wrappers to the
    │                                   concrete-typed BrakingController constructor
    │
    ├── TestScenario   ← declarative test definition (Builder pattern)
    │
    └── TestResult / TestReport  ← result and formatted report
```

### Data flow

1. `AEBSTestSuite.buildAllScenarios()` builds a list of `TestScenario` objects.
2. `AEBSTestRunner.runAll()` iterates the list and calls `runScenario()` on each.
3. For each scenario, `runScenario()`:
   - Constructs 13 sensor instances, wrapping any that have a non-NONE fault.
   - Builds a `BrakingControllerTestAdapter` (which calls the real `BrakingController`
     via shim sensors).
   - Starts all sensors and the controller, then polls `getLastDecision()` every 50 ms.
   - Stops everything cleanly, then evaluates assertions.
4. A `TestReport` is written to `aebs_test_report.txt`.

---

## How to run

```bash
# From the project root (where the `scenarios/` folder lives):
javac -cp .:lib/json.jar \
      src/swen326/group4/**/*.java \
      src/swen326/group4/test/*.java

java -cp .:lib/json.jar swen326.group4.test.AEBSTestSuite
```

The runner prints progress to `stderr` and writes a full report to
`aebs_test_report.txt`. Exit code is `0` if all tests pass, `1` if any fail.

---

## Adding a new test

Open `AEBSTestSuite.java` and add a new `TestScenario.Builder` call in the
appropriate group method (or create a new group):

```java
g.add(new TestScenario.Builder("TC-NNN", "scenarioFolderName", durationMs)
    .description("What this test checks and why")

    // -- Optional: inject faults --
    .faultLidar1(FaultType.HARD_FAIL)
    .faultRadar2(FaultType.DEGRADED)
    .faultDriver(FaultType.SILENT)

    // -- Optional: configure delayed-fail --
    .delayLidar1(10)   // fail after 10 ticks (~500ms at 50ms/tick)

    // -- Expected outcomes --
    .expectDecisionAtLeastOnce(ControllerDecision.AUTONOMOUS_BRAKE)
    .forbidDecision(ControllerDecision.SENSOR_FAULT)
    .expectBrakingEngaged(true)

    .build());
```

### Scenario data folders

Each `scenarioId` maps to `scenarios/<scenarioId>/` under the project root.
The folder must contain the sensor data files expected by the sensors you leave
healthy (fault-injected sensors never open files).

For the existing `world1` scenario the required files are:
```
scenarios/world1/
  1worldCameraData.json
  2worldCameraData.json
  3worldCameraData.json
  1worldRadarData.json
  2worldRadarData.json
  3worldRadarData.json
  1worldLidarData.json
  2worldLidarData.json
  3worldLidarData.json
  worldWheelSpeedId1.json
  worldWheelSpeedId2.json
  worldWheelSpeedId3.json
  worldDriverData.json
```

If a sensor is fault-injected (any fault except `NONE`), its data file is
never opened, so you only need files for healthy sensors.

---

## Fault types

| FaultType       | Behaviour                                                  | Use case                          |
|-----------------|-------------------------------------------------------------|-----------------------------------|
| `NONE`          | Real sensor, reads from file normally                       | Baseline                          |
| `HARD_FAIL`     | Returns `null` readings, status = `FAILED` immediately      | FR2104 — sensor totally offline   |
| `SILENT`        | Returns `null` readings, status = `FAILED`                  | Sensor powered but silent         |
| `FROZEN_READING`| Returns the same reading forever, status = `OK`             | Stuck sensor (FR4203)             |
| `DEGRADED`      | Returns low-confidence readings, status = `DEGRADED`        | FR2103 — weather degradation      |
| `DELAYED_FAIL`  | Works for N ticks then hard-fails (use `delayXxx()` setter) | Mid-scenario failure              |
| `INTERMITTENT`  | Alternates null/valid each tick                             | Flaky sensor                      |

---

## Assertions available on TestScenario.Builder

| Method                              | Assertion                                                       |
|-------------------------------------|-----------------------------------------------------------------|
| `expectDecisionAtLeastOnce(D)`      | Decision `D` must appear at least once during the run           |
| `forbidDecision(D)`                 | Decision `D` must never appear                                  |
| `expectBrakingEngaged(true)`        | `BrakeActuator.getCurrentIntensity()` must exceed 0.0 at some point |
| `expectNoBraking(true)`             | `BrakeActuator` must stay at 0.0 throughout                     |

Multiple `expectDecisionAtLeastOnce` and `forbidDecision` calls can be chained —
all conditions must hold simultaneously.

---

## Test catalogue summary

| ID     | Description                                              | Requirements        |
|--------|----------------------------------------------------------|---------------------|
| TC-001 | All sensors healthy, approaching vehicle → brake         | FR3101, TTC logic   |
| TC-002 | No hazard, stay CLEAR                                    | No false positive   |
| TC-101 | Lidar1 HARD_FAIL → 2oo3 still brakes                    | FR2104              |
| TC-102 | Lidar2 HARD_FAIL → 2oo3 still brakes                    | FR2104              |
| TC-103 | Radar1 HARD_FAIL → 2oo3 still brakes                    | FR2107              |
| TC-104 | Radar3 HARD_FAIL → 2oo3 still brakes                    | FR2107              |
| TC-105 | Camera1 HARD_FAIL → ranging still brakes                | NF2202              |
| TC-106 | WheelSensor1 HARD_FAIL → wheel vote still works         | FR2208              |
| TC-107 | Lidar1 DEGRADED → weight shifts to Radar                | FR2103              |
| TC-108 | Radar1 DEGRADED → weight shifts to Lidar                | FR2103              |
| TC-109 | Lidar1 FROZEN → outlier excluded, 2oo3 brakes           | FR4203              |
| TC-201 | Lidar1+2 HARD_FAIL → SENSOR_FAULT, no brake             | FR2104, FR2106      |
| TC-202 | Radar1+2 HARD_FAIL → SENSOR_FAULT, no brake             | FR2107              |
| TC-203 | Camera1+2 HARD_FAIL → SENSOR_FAULT raised               | NF2202              |
| TC-204 | WheelSensor1+2 HARD_FAIL → escalation                   | FR2208              |
| TC-205 | All Lidars HARD_FAIL → SENSOR_FAULT, no brake           | FR2106              |
| TC-301 | Driver always braking → DRIVER_OVERRIDE, no auto-brake  | FR3104              |
| TC-302 | Inattentive driver → AUTONOMOUS_BRAKE fires              | HF-002              |
| TC-303 | Driver sensor failed → AUTONOMOUS_BRAKE fires            | HF-001              |
| TC-401 | Lidar1 delayed-fail mid-run → brake still engages        | FR2104              |
| TC-402 | Radar1 delayed-fail mid-run → brake still engages        | FR2107              |
| TC-403 | Lidar1 intermittent → 2oo3 tolerates flicker             | FR2104              |
| TC-404 | Brake retries exhausted → ESCALATION fires               | FR-3105, FR-3106    |
| TC-405 | Camera1 delayed-fail → SENSOR_FAULT then brake          | NF2202              |
| TC-406 | Lidar1+Radar2 HARD_FAIL → cross-modality, brake fires   | FR2104, FR2107      |
| TC-407 | WheelSensor1 FROZEN → sentinel excluded, brake fires    | FR4203              |
