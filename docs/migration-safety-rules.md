# Java Migration Safety Rules

Current user override: `cpp/` is the real robot code and the source of truth.
The `java/` directory is only a random Java reference project. Java template
parameters, mechanism structure, controller bindings, and robot behavior must
not be trusted when they conflict with C++.

Manual migration from C++ to Java is allowed for this project, including
generated swerve constants, because the C++ code is the real machine baseline.
When a later Tuner X Java project is available, compare it against C++ facts
before replacing the manually migrated Java constants.

## Active Rules

- Keep `cpp/` read-only unless the user explicitly asks otherwise.
- Migrate robot behavior, CAN IDs, CAN buses, inversion, neutral modes, current
  limits, sensor ratios, PID gains, homing, stop/hold behavior, controller
  bindings, autonomous fallback, and command safety from C++.
- Treat `java/` as API/style reference only.
- Keep controllers and trigger bindings in `RobotContainer`.
- Preserve release cleanup, interruption safety, timeouts, Do Nothing
  autonomous, alliance unknown fallback, and PathPlanner failure fallback.
- Validate each migration phase with the Java Gradle build before widening the
  scope.
- Do not run old C++ project scripts for the Java port.

