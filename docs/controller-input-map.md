# Controller Input Map

## Purpose
This file records controller input ownership to avoid duplicate bindings.
When adding new bindings in `RobotContainer`, check this file first.

## Subsystem Periodic Ownership
- `ShooterSubsystem::Periodic` (`src/main/cpp/subsystems/ShooterSubsystem.cpp`)
  - `A` button: shoot/follow feeder logic
  - `LeftTrigger` / `RightTrigger`: linear servo incremental control
- `FeederSubsystem::Periodic` (`src/main/cpp/subsystems/FeederSubsystem.cpp`)
  - `X` button: feeder combo velocity control
- `GroundIntakeSubsystem::Periodic` (`src/main/cpp/subsystems/GroundIntakeSubsystem.cpp`)
  - `POVUp`: set pitch to preset
  - `POVDown`: set pitch to preset

## RobotContainer Ownership
- `LeftBumper`: toggle drivetrain open/closed loop
- `RightBumper`: drive aiming command
- `Y`: toggle ground intake prepare/reset
- `B`: ground intake assist command
- `Start`: feeder preload command
- `LeftStick` / `RightStickX`: drivetrain default drive

## Current Conflict Status
- No direct button conflict between active `RobotContainer` bindings and periodic-owned buttons.

## Update Rule
- If a button is added in any `Subsystem::Periodic`, update this file.
- If a button is rebound in `RobotContainer`, re-check this file and update.
