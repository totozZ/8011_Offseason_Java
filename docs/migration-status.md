# Java Migration Status

The active migration branch is `java-port`. The C++ snapshot remains the robot
source of truth and has not been modified. The untracked `java/` directory is
still only a temporary API reference.

## Completed

- Phase 0: root Java GradleRIO project and branch structure.
- Phase 1: PathPlanner dependency and C++ deploy assets.
- Phase 2: manual C++-to-Java swerve constants alignment.
- Phase 3: Robot lifecycle, RobotContainer bindings, NamedCommands, safe
  Do Nothing autonomous, alliance fallback, and release cleanup.
- Phase 4: Shooter, feeder, ground intake, and WayiMotor behavior.
- Phase 5: shooting, passing, intake routes, dynamic paths, and
  shoot-on-move path generation.
- Phase 6: Limelight vision fusion, client dashboard telemetry, and dormant
  CANdle LED implementation.
- Follow-up safety audit: restored the 6.5 V brownout threshold, teleop swerve
  drive-motor Brake neutral mode, and C++-equivalent telemetry load.

## Intentionally Dormant

- `LEDSubsystem` is migrated but not constructed because the C++ RobotContainer
  also does not construct it.
- AprilTags, GPDetection, event-marker path overloads, drive-current mutation,
  and mechanism SysId helpers have no active C++ caller or binding.
- The only deployed PathPlanner auto is the safe `Do Nothing` auto, matching
  the C++ deploy baseline.

## Required Before Cleanup

- Regenerate or independently verify the Java Tuner output against the C++
  constants.
- Bench-check CANivore, Pigeon 33, swerve IDs 1-12, module offsets, inversion,
  current limits, and Brake/Coast behavior.
- Validate shooter and ground-intake homing direction, current thresholds,
  timeout behavior, and mechanical zero positions.
- Validate controller release/interruption cleanup and all pass/intake routes.
- Validate PathPlanner coordinates, red alliance behavior, dynamic path failure
  fallback, and end-state stopping.
- Validate all three Limelight names, MegaTag timestamps, pose frames,
  rejection thresholds, and odometry fusion.

Do not remove `cpp/` or `java/`, or merge back to `main`, until the bench
validation checkpoint is explicitly accepted.
