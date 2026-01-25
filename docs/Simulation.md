# Simulation Scope (2026 Template)

## Platform Choice
- Use WPILib Simulation + CTRE Phoenix 6 simulation with PathPlanner.
- AdvantageScope is used for visualization and log verification (not the physics engine).

## Scope
- Only test the swerve subsystem in simulation.

## Acceptance Criteria
- Under PathPlanner auto paths, the drivetrain pose moves as expected in AdvantageScope.
- Verify multiple auto strategies (at least 2-3 distinct auto paths).

## Test Plan
1) Run simulation with the default PathPlanner auto.
2) Load and run additional auto paths (e.g., left/right/middle) and verify pose tracks.
3) Capture logs and confirm path tracking in AdvantageScope.
