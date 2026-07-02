# Swerve Tuner Conflict Gate

This file records the Phase 2 migration rule for the Java port. The C++
project is the robot-parameter source of truth. The Java template is only a
random reference project and must not be trusted for robot parameters.

User authorization update: manual migration from C++ generated swerve constants
into Java is allowed for this project because the C++ code is the real robot
code. Regenerate with Tuner X later when available, and compare the regenerated
Java output against the C++ facts before robot testing.

## Current Status

- C++ source of truth: `cpp/src/main/include/generated/TunerConstants.h`
- Java generated file currently present:
  `src/main/java/frc/robot/generated/TunerConstants.java`
- The active Java Tuner constants have been manually aligned to the C++ robot
  constants after explicit user authorization.
- Remaining risk: this is not an official Tuner X Java regeneration. It must be
  validated on blocks / real robot before drivetrain motion.

## Conflicts Resolved By Manual Migration

| Area | C++ source of truth | Original Java template |
| --- | --- | --- |
| CAN bus | `CANivore` | empty bus name / default bus |
| Pigeon2 ID | `33` | `13` |
| Speed at 12 V | `4.59 m/s` | `0.8 m/s` |
| Coupling ratio | `3.5714285714285716` | `2.8333333333333335` |
| Drive gear ratio | `6.746031746031747` | `6.538461538461539` |
| Steer gear ratio | `21.428571428571427` | `15.42857142857143` |
| Wheel radius | `2.008 in` | `2 in` |
| Drive current limits | supply `40 A`, stator `90 A`, both enabled | supply `70 A`, no stator limit in active config |
| Steer current limits | supply `20 A`, stator `60 A`, both enabled | stator `60 A`, no supply limit in active config |
| Drive gains | `kS=0.23`, `kV=0.123`, `kA=0.1`, `kP=0.1` | `kS=0`, `kV=0.124`, no active `kA`, `kP=0.1` |
| Steer gains | `kV=2.66`, `kP=100`, `kD=0.5`, `kS=0.1` | `kV=1.91`, `kP=100`, `kD=0.5`, `kS=0.1` |
| Module positions | `+/-10.875 in` | `+/-10 in` |

## Module-Level Conflicts Resolved By Manual Migration

| Module | C++ drive / steer / encoder / offset | Original Java template values |
| --- | --- | --- |
| Front Left | `2 / 1 / 3 / -0.376952 rot` | `21 / 22 / 23 / 0.16015625 rot` |
| Front Right | `5 / 4 / 6 / 0.197021484375 rot` | `24 / 25 / 26 / 0.209228515625 rot` |
| Back Left | `8 / 7 / 9 / -0.379150 rot` | `27 / 28 / 29 / -0.399658203125 rot` |
| Back Right | `11 / 10 / 12 / -0.35205078125 rot` | `30 / 31 / 32 / -0.02392578125 rot` |

## Notes

- `RobotHardware.md` in the Java template mentions a `1-12` ID set, but it
  conflicts with the active Java constants and with the C++ generated
  drive/steer ordering. Treat the C++ generated values as the current source of
  truth until hardware re-verification says otherwise.
- The Java template had comments about front-left drive inversion. The manual
  migration follows the active C++ generated constants, where front-left drive
  uses `kInvertLeftSide=false`. This remains a real-robot validation item.
- Phoenix version is also different: C++ uses vendordep `26.1.1`, Java template
  uses `26.3.0`. The current migration default is to keep Java `26.3.0` unless
  the regenerated Tuner X Java project requires a different Phoenix vendordep.

## Required Next Validation

- Build must pass after the manual migration.
- Before robot motion, verify CANivore presence, Pigeon ID `33`, module IDs,
  module angle offsets, drive/steer inversion, and neutral/current behavior.
- When a real Tuner X Java project becomes available, replace this manual
  migration with generated Java output and compare all constants before merging
  back to `main`.
