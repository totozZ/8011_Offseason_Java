# Controller Input Map

Java port status: controller bindings and Phase 5 command migration complete.

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
| Right Bumper | C++ PassBump route, chosen by opponent-route heuristic |
| Y | Stop shooter/feeder, then C++ PassTrench route chosen by opponent-route heuristic |
| A | C++ intake-next-to-side route while held |
| B | C++ intake-next-to-hub/wall route on opponent side while held |
| X | C++ intake-next-to-hub/wall route on own side while held |

## Notes

- The random Java template bindings for brake, point wheels, SysId, and shooter
  spin-up were removed because the preserved `cpp` branch is the real robot
  source of truth.
- Shooter, feeder, ground-intake cleanup, pass routes, intake routes, dynamic
  PathPlanner generation, and shoot-on-move helpers are migrated from C++.
- Compile-time and pure-logic tests do not replace real controller and
  interruption testing on the robot.
