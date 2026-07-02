# Controller Input Map

Java port status: Phase 3 safe drivetrain/autonomous skeleton.

## Driver Controller Port 0

| Input | Current Java behavior |
| --- | --- |
| Left Y | Field-centric X velocity, scaled by `OperatorConstants.speedRate` |
| Left X | Field-centric Y velocity, scaled by `OperatorConstants.speedRate` |
| Right X | Rotational velocity, scaled by `OperatorConstants.angularSpeedRate` |
| All buttons/triggers/POV | Unbound until the related C++ subsystems and commands are migrated |

## Notes

- The random Java template bindings for brake, point wheels, SysId, and shooter
  spin-up were removed because `cpp/` is the real robot source of truth.
- C++ mechanism actions, release cleanup, and shooting/intake command behavior
  must be restored in later phases only after the matching Java subsystems
  exist.
