#include "subsystems/ShooterSubsystem.h"

using namespace subsystems;

void ShooterSubsystem::Initialization()
{
  configs::TalonFXConfiguration shooter_left_front_config{};
  shooter_left_front_config.MotorOutput.Inverted = 0;
  /* slot0 PID槽*/
  configs::Slot0Configs& shooter_left_front_slot0 = shooter_left_front_config.Slot0;
  shooter_left_front_slot0.kG = 0.;          // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  shooter_left_front_slot0.kS = 0.12;        // Add 0.25 V output to overcome static friction
  shooter_left_front_slot0.kV = 0.12;        // A velocity target of 1 rps results in 0.12 V output
  shooter_left_front_slot0.kA = 0;           // An acceleration of 1 rps/s requires 0.01 V output
  shooter_left_front_slot0.kP = 0.03;        // A position error of 0.2 rotations results in 12 V output
  shooter_left_front_slot0.kI = 0;           // No output for integrated error
  shooter_left_front_slot0.kD = 0.;          // A velocity error of 1 rps results in 0.5 V output
  shooter_left_front_slot0.GravityType = 0;  // elevator重力补偿

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode shooter_left_front_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    shooter_left_front_status = shooter_left_front_.Applyconfig(shooter_left_front_config);
    if (shooter_left_front_status.IsOK())
      break;
  }
  shooter_left_front_.setgearRatio(1.3);

  shooter_right_.setfollowControl(shooter_left_front_.Getdata().deviceId, false);
  shooter_right_.setgearRatio(1.3);

  frc::SmartDashboard::PutNumber("Shooter/Target X Distance:", 0.);
  frc::SmartDashboard::PutNumber("Shooter/Speed_conversion_eff:", 0.);
}

void ShooterSubsystem::Periodic()
{
  shooter_left_front_.Receive();
  shooter_left_front_.Control();

  shooter_right_.Receive();
  shooter_right_.Control();

  shoot_vel_ = frc::SmartDashboard::GetNumber("Shooter/Shoot Velocity:", 5.);
  speed_conversion_efficiency_ = frc::SmartDashboard::GetNumber("Shooter/Speed_conversion_eff:", 0.3);

  Eigen::Vector3d pos = { 0.3, 0., ShooterConstants::TargetHeight - ShooterConstants::ShooterHeight };
  Eigen::Vector3d vel = { 0., 0., 0. };
  pos[0] = frc::SmartDashboard::GetNumber("Shooter/Target X Distance:", 0.3);
  frc::SmartDashboard::PutNumber("Shooter/Target Height:", pos[2]);
  ball_solver_.setTarget(pos, vel);

  if (ball_solver_.solveForSpeed((61.818 - 3.636 * pos[0]) * M_PI / 180.0, true))
  {
    shoot_vel_ = ball_solver_.getBallSpeed();
    frc::SmartDashboard::PutString("Shooter/Solve Status:", "Success");
  }
  else
    frc::SmartDashboard::PutString("Shooter/Solve Status:", "Failed");

  frc::SmartDashboard::PutNumber("Shooter/Shoot Velocity:", shoot_vel_);
  frc::SmartDashboard::PutNumber("Shooter/Shoot Angle:", 61.818 - 3.636 * pos[0]);
  frc::SmartDashboard::PutNumber("Shooter/Motor target Velocity:",
                                 shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                                     speed_conversion_efficiency_ / shooter_left_front_.Getdata().gearRatio);
  if (m_joystick.LeftBumper().Get())
  {
    SetShootVelocity(shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) / speed_conversion_efficiency_ /
                     shooter_left_front_.Getdata().gearRatio);
  }

  if (m_joystick.RightBumper().Get())
  {
    // Shoot(shoot_vel_, shoot_pitch_angle_);
    SetShootVelocity(shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                     ShooterConstants::SpeedConversionEfficiency /

                     shooter_left_front_.Getdata().gearRatio);
  }
  else if (m_joystick.X().Get())
  {
    Stop();
  }
}
void ShooterSubsystem::SetShootVelocity(double velocity)
{
  velocity /= shooter_left_front_.Getdata().gearRatio;
  shooter_left_front_.setvelocity(velocity);
}

frc2::CommandPtr ShooterSubsystem::SetShootVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetShootVelocity(velocity); });
}

double ShooterSubsystem::GetShootVelocity()
{
  return shooter_left_front_.Getdata().currentVelocity;
}

void ShooterSubsystem::Shoot(double shoot_vel, double shoot_angle)
{
  double shoot_wheel_velocity = shoot_vel / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                                ShooterConstants::SpeedConversionEfficiency /
                                shooter_left_front_.Getdata().gearRatio;  // 转换为转每秒
  SetShootVelocity(shoot_wheel_velocity);
}

void ShooterSubsystem::Stop()
{
  SetShootVelocity(0.);
}