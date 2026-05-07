# HANDOFF.md — Project Status & Breadcrumbs

This file is maintained by the person who bootstrapped the project.
Update it as you make progress so the group stays oriented.

## Current Status

Bootstrapped by [Your Name] on [Date].
Group development is behind schedule — priority is getting something 
critiqueable before the Week 11 in-person assessment.

## What Exists So Far

- Folder structure
- README.md
- This HANDOFF.md

## What Still Needs Doing (Priority Order)

### Documentation (do these first)
- [ ] `docs/GroupAgreement/` — Group working agreement, signed by all members
- [ ] `docs/ProjectPlan/` — Automotive Safety Plan, HARA, ASIL ratings
- [ ] `docs/RequirementsTrace/` — Requirements list (REQ-001, REQ-002...) and traceability matrix
- [ ] `docs/FTA/` — Fault Tree Analysis (at minimum: "Failure to brake", "Sensor failure")

### Core System (`core/`)
- [ ] `SensorInput.java` — Handles radar/lidar, camera, wheel speed inputs
- [ ] `RedundancyManager.java` — Voting logic between redundant sensors
- [ ] `BrakingDecisionEngine.java` — Core logic: when and how hard to brake
- [ ] `DriverAlertInterface.java` — Console output for alerts and system status
- [ ] `AEBSController.java` — Main controller tying everything together

### Simulator (`simulator/`)
- [ ] `SensorSimulator.java` — Generates fake sensor data
- [ ] `ScenarioRunner.java` — Runs named scenarios (see below)
- [ ] Scenarios to implement:
  - Normal highway driving
  - Sudden obstacle (stationary object)
  - Pedestrian crossing
  - Sensor failure (one redundant sensor dies)
  - Vehicle braking hard in front

### Tests (`tests/`)
- [ ] Unit tests for `BrakingDecisionEngine.java`
- [ ] Unit tests for `RedundancyManager.java`
- [ ] At least one fault condition test (sensor returns null or out-of-range value)
- [ ] At least one full scenario test using the simulator

## Requirements Summary

Requirements are numbered REQ-001 onwards and defined in full in
`docs/RequirementsTrace/`. Key ones to know:

| ID | Summary |
|----|---------|
| REQ-001 | System shall detect objects 0.5–200m via radar/lidar |
| REQ-002 | Radar/lidar shall update every 100ms |
| REQ-003 | Camera shall classify objects every 50ms |
| REQ-004 | Wheel speed sensors shall update every 10ms |
| REQ-005 | All sensors shall have redundancy |
| REQ-006 | Braking control signals sent at least every 50ms during active braking |
| REQ-007 | Brake execution verified within 50ms via sensor feedback |
| REQ-008 | Success = wheel speed reduction within ±5% of expected deceleration |
| REQ-009 | On failure, retry braking up to 2 more times before driver alert |
| REQ-010 | Driver interface shall provide auditory and visual alerts |
| REQ-011 | System shall be manually activatable/deactivatable |

## Traceability Reminder

Every piece of code should trace back to a REQ-XXX.
Every REQ-XXX should trace forward to a test.
This is what the in-person marking will focus on.

## Notes for Group Members

- Use AI tools freely — you won't be marked on quality, only your ability to critique
- Keep commits small and frequent — it shows ongoing engagement
- If you add a file, update this HANDOFF.md
- Ask questions on the Nuku forum, not via email to Stuart