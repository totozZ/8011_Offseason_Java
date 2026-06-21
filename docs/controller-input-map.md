# Controller Input Map

Controller input is owned by `RobotContainer`; no subsystem reads the driver
controller from `Periodic()`.

## RobotContainer Ownership

- `RightTrigger`: Hub or automatic region Pass shot
- `RightBumper`: PassBump tactical movement
- `Y`: PassTrench tactical movement
- `POVUp`: zero-pitch fallback shot
- `POVDown`: Tower fallback shot
- `POVRight`: reverse ground intake
- `LeftTrigger`: prepare ground intake
- `A`: side intake route
- `B`: Hub/wall intake route
- `X`: opposite Hub/wall intake route
- `Start`: retract/reset ground intake

| Input | Active behavior |
| --- | --- |
| Right Trigger | Hub lookup-table shot or automatic region Pass shot |
| Right Bumper | PassBump tactical movement |
| Y | PassTrench tactical movement |
| POV Up | Re-home pitch, then zero-pitch fallback shot |
| POV Down | Re-home pitch, then 3.048 m Tower fallback shot |
| POV Right | Reverse ground intake while held |
| Left Trigger | Prepare ground intake while held |
| A | Side intake route |
| B / X | Hub or wall intake route, opposite variants |
| Start | Retract/reset ground intake |
| Left Stick / Right Stick X | Default drivetrain control |

Main shooting is blocked when alliance is unknown. The two POV fallback shots
remain available. Shooting commands own Shooter and Feeder; aiming commands own
the drivetrain; release paths stop feeding and restore flywheel idle.
