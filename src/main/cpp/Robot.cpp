// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

#include "LimelightHelpers.h"

Robot::Robot() {}

void Robot::RobotPeriodic() {
  static constexpr double kSchedulerOverrunMs = 20.0;
  static int scheduler_overrun_count = 0;
  static double scheduler_loop_ms_max = 0.0;
  static double last_epoch_print_s = -1.0;

  auto& scheduler = frc2::CommandScheduler::GetInstance();
  const double loop_start_s = frc::Timer::GetFPGATimestamp().value();

  m_container.UpdateDriverPerspective();

  scheduler.Run();

  const double loop_end_s = frc::Timer::GetFPGATimestamp().value();
  const double scheduler_loop_ms = (loop_end_s - loop_start_s) * 1000.0;
  if (scheduler_loop_ms > scheduler_loop_ms_max) {
    scheduler_loop_ms_max = scheduler_loop_ms;
  }
  frc::SmartDashboard::PutNumber("SchedulerLoopMs", scheduler_loop_ms);
  frc::SmartDashboard::PutNumber("SchedulerLoopMsMax", scheduler_loop_ms_max);
  frc::SmartDashboard::PutNumber("SchedulerLoopOverrunThresholdMs",
                                 kSchedulerOverrunMs);
  frc::SmartDashboard::PutBoolean("SchedulerLoopOverrun",
                                  scheduler_loop_ms > kSchedulerOverrunMs);

  if (scheduler_loop_ms > kSchedulerOverrunMs) {
    ++scheduler_overrun_count;
    // Print watchdog epochs to locate which Periodic/Execute section is slow.
    if (last_epoch_print_s < 0.0 || (loop_end_s - last_epoch_print_s) > 0.5) {
      scheduler.PrintWatchdogEpochs();
      last_epoch_print_s = loop_end_s;
    }
  }
  frc::SmartDashboard::PutNumber("SchedulerLoopOverrunCount",
                                 scheduler_overrun_count);
}

void Robot::DisabledInit() {
  // Seed IMU
  //m_container.clientSub.PubRobotInit(0);
  m_container.visionSub.disable_mix = 1;
}

void Robot::DisabledPeriodic() {
    m_container.refreshAutoMode();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  nt::NetworkTableInstance::GetDefault()
      .GetTable("limelight-front")
      ->PutNumber("throttle_set", 0);
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  try {
  } catch (const std::exception& e) {
    std::cout << "auto Periodic Failed: " << e.what() << std::endl;
  }
}

void Robot::AutonomousExit() {
  m_container.feederSub.Stop();
  m_container.shooterSub.DisableShooterNonCmd();
  m_container.groundIntakeSub.Stop();
  m_container.drivetrain.SetControl(m_container.drivetrain.Idle);
}

void Robot::TeleopInit() {
  nt::NetworkTableInstance::GetDefault()
      .GetTable("limelight-front")
      ->PutNumber("throttle_set", 0);
  // m_container.clientSub.PubRobotInit(1);
  m_container.groundIntakeSub.SetTeleopRollerCurrentLimit();
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value().get());
  }
  m_container.visionSub.disable_mix = 0;
}

void Robot::TeleopPeriodic() {
  try {
  } catch (const std::exception& e) {
    std::cout << "pathplanner Periodic Failed: " << e.what() << std::endl;
  }
}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {
  frc::SmartDashboard::PutData("Field", &m_simField);
}

void Robot::SimulationPeriodic() {
  m_simField.SetRobotPose(m_container.drivetrain.GetState().Pose);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
