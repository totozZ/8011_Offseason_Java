# Controller Input Map

Java port status: Phase 5 shooting command slice.

## Driver Controller Port 0

| Input | Current Java behavior |
| --- | --- |
| Left Y | Field-centric X velocity, scaled by `OperatorConstants.speedRate` |
| Left X | Field-centric Y velocity, scaled by `OperatorConstants.speedRate` |
| Right X | Rotational velocity, scaled by `OperatorConstants.angularSpeedRate` |
| Start | Ground intake cleanup, then pitch to `0.07` normalized |
| Right Trigger | Hub/pass shooting command selected from field region; blocked if alliance is unknown |
| Left Trigger | Ground intake prepare while held; cleanup on release |
| POV Up | Zero-pitch fallback shot while held; cleanup on release |
| POV Down | Tower fallback shot while held; cleanup on release |
| POV Right | Ground intake anti command on press; cleanup on release |
| Right Bumper / A / B / X / Y | Still unbound until C++ AutoMove/intake route commands are migrated |

## Notes

- The random Java template bindings for brake, point wheels, SysId, and shooter
  spin-up were removed because `cpp/` is the real robot source of truth.
- Shooter, feeder, and ground-intake cleanup now follows the first migrated C++
  command slice. Remaining driver tactical movement and route commands still
  need their C++ AutoMove dependencies migrated before binding.
