#include "subsystems/ShooterSubsystem.h"

using namespace subsystems;

void ShooterSubsystem::Initialization()
{
  // 云台俯仰电机配置
  configs::TalonFXConfiguration pitch_config{};
  pitch_config.MotorOutput.Inverted = 0;
  /* Configure Motion Magic */
  configs::MotionMagicConfigs& pitch_mm = pitch_config.MotionMagic;
  pitch_mm.MotionMagicAcceleration = 10 * 16.3636_tr_per_s_sq;  // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel
  pitch_mm.MotionMagicJerk = 100 * 16.3636_tr_per_s_cu;
  pitch_mm.MotionMagicCruiseVelocity = 0_tps;           // 5 (mechanism) rotations per second cruise
  pitch_mm.MotionMagicExpo_kV = 0.1_V / 1_tps;          // 0.12
  pitch_mm.MotionMagicExpo_kA = 0.1_V / 1_tr_per_s_sq;  // 0.1
  /* slot0 PID槽*/
  configs::Slot0Configs& pitch_slot0 = pitch_config.Slot0;
  pitch_slot0.kG = 0.;  // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  pitch_slot0.kS = 0.;  // Add 0.25 V output to overcome static friction
  pitch_slot0.kV = 0;   // A velocity target of 1 rps results in 0.12 V output
  pitch_slot0.kA = 0;   // An acceleration of 1 rps/s requires 0.01 V output
  pitch_slot0.kP = 3;   // A position error of 0.2 rotations results in 12 V output
  pitch_slot0.kI = 0;   // No output for integrated error
  pitch_slot0.kD = 0;   // A velocity error of 1 rps results in 0.5 V output
  pitch_slot0.GravityType = 1;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode pitch_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    pitch_status = pitch_.Applyconfig(pitch_config);
    if (pitch_status.IsOK())
      break;
  }
  // pitch_.Reset(pitch_.Getmotor().GetPosition().GetValueAsDouble());

  configs::TalonFXConfiguration shooter_left_config{};
  shooter_left_config.MotorOutput.Inverted = 0;
  /* slot0 PID槽*/
  configs::Slot0Configs& shooter_left_slot0 = shooter_left_config.Slot0;
  shooter_left_slot0.kG = 0.;          // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  shooter_left_slot0.kS = 0.12;        // Add 0.25 V output to overcome static friction
  shooter_left_slot0.kV = 0.12;        // A velocity target of 1 rps results in 0.12 V output
  shooter_left_slot0.kA = 0;           // An acceleration of 1 rps/s requires 0.01 V output
  shooter_left_slot0.kP = 0.03;        // A position error of 0.2 rotations results in 12 V output
  shooter_left_slot0.kI = 0;           // No output for integrated error
  shooter_left_slot0.kD = 0.;          // A velocity error of 1 rps results in 0.5 V output
  shooter_left_slot0.GravityType = 0;  // elevator重力补偿

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode shooter_left_status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    shooter_left_status = shooter_left_.Applyconfig(shooter_left_config);
    if (shooter_left_status.IsOK())
      break;
  }
  shooter_left_.setgearRatio(1.3);

  shooter_right_.setfollowControl(shooter_left_.Getdata().deviceId, false);
  shooter_right_.setgearRatio(1.3);

  frc::SmartDashboard::PutNumber("Shooter/Target X Distance:", 0.);
  frc::SmartDashboard::PutNumber("Shooter/Speed_conversion_eff:", 0.);
}

void ShooterSubsystem::Periodic()
{
  pitch_.Receive();
  pitch_.Control();
  shooter_left_.Receive();
  shooter_left_.Control();
  shooter_right_.Receive();
  shooter_right_.Control();

  shoot_vel_ = frc::SmartDashboard::GetNumber("Shooter/Shoot Velocity:", 5.);
  shoot_pitch_angle_ = frc::SmartDashboard::GetNumber("Shooter/Shoot Angle:", 50);
  speed_conversion_efficiency_ = frc::SmartDashboard::GetNumber("Shooter/Speed_conversion_eff:", 0.3);

  Eigen::Vector3d pos = { 0.3, 0., ShooterConstants::TargetHeight - ShooterConstants::ShooterHeight };
  Eigen::Vector3d vel = { 0., 0., 0. };
  pos[0] = frc::SmartDashboard::GetNumber("Shooter/Target X Distance:", 0.3);
  frc::SmartDashboard::PutNumber("Shooter/Target Height:", pos[2]);
  ball_solver_.setTarget(pos, vel);

  // if (ball_solver_.solveForGimBalAngle(shoot_vel_))
  // {
  //   frc::SmartDashboard::PutString("Shooter/Solve Status:", "Success");
  //   shoot_pitch_angle_ = ball_solver_.getSolvedPitch() * 180.0 / M_PI;
  // }
  // else
  // {
  //   // shoot_pitch_angle_ = Shooter::PitchMaxAngle;
  //   frc::SmartDashboard::PutString("Shooter/Solve Status:", "Failed");
  // }

  if (ball_solver_.solveForSpeed((61.818 - 3.636 * pos[0]) * M_PI / 180.0, true))
  {
    shoot_vel_ = ball_solver_.getBallSpeed();
    frc::SmartDashboard::PutString("Shooter/Solve Status:", "Success");
  }
  else
    frc::SmartDashboard::PutString("Shooter/Solve Status:", "Failed");

  frc::SmartDashboard::PutNumber("Shooter/Shoot Velocity:", shoot_vel_);
  // frc::SmartDashboard::PutNumber("Shooter/Shoot Angle:", shoot_pitch_angle_);
  frc::SmartDashboard::PutNumber("Shooter/Shoot Angle:", 61.818 - 3.636 * pos[0]);
  frc::SmartDashboard::PutNumber("Shooter/Pitch position:", (ShooterConstants::PitchMaxAngle - shoot_pitch_angle_) *
                                                                ShooterConstants::PitchDisplacementPerDegree);
  frc::SmartDashboard::PutNumber("Shooter/Motor target Velocity:",
                                 shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                                     speed_conversion_efficiency_ / shooter_left_.Getdata().gearRatio);
  if (m_joystick.LeftBumper().Get())
  {
    SetShootVelocity(shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) / speed_conversion_efficiency_ /
                     shooter_left_.Getdata().gearRatio);
  }

  if (m_joystick.RightBumper().Get())
  {
    // Shoot(shoot_vel_, shoot_pitch_angle_);
    SetPitchPosition((ShooterConstants::PitchMaxAngle - shoot_pitch_angle_) *
                     ShooterConstants::PitchDisplacementPerDegree);
    if (m_joystick.A().Get())
    {
      SetShootVelocity(shoot_vel_ / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                       ShooterConstants::SpeedConversionEfficiency /

                       shooter_left_.Getdata().gearRatio);
    }
  }
  else if (m_joystick.X().Get())
  {
    Stop();
    SetPitchPosition(ShooterConstants::PitchMinPosition);
  }
}

void ShooterSubsystem::SetPitchPosition(double position)
{
  if (position > ShooterConstants::PitchMaxPosition)
    position = ShooterConstants::PitchMaxPosition;
  else if (position < ShooterConstants::PitchMinPosition)
    position = ShooterConstants::PitchMinPosition;

  pitch_.setmotionposition(position);
}

void ShooterSubsystem::SetShootVelocity(double velocity)
{
  velocity /= shooter_left_.Getdata().gearRatio;
  shooter_left_.setvelocity(velocity);
}

frc2::CommandPtr ShooterSubsystem::SetPitchPositionCommandPtr(double position)
{
  return frc2::cmd::RunOnce([this, position] { SetPitchPosition(position); });
}

frc2::CommandPtr ShooterSubsystem::SetShootVelocityCommandPtr(double velocity)
{
  return frc2::cmd::RunOnce([this, velocity] { SetShootVelocity(velocity); });
}

double ShooterSubsystem::GetShootVelocity()
{
  return shooter_left_.Getdata().currentVelocity;
}

double ShooterSubsystem::GetPitchPosition()
{
  return pitch_.Getdata().currentPosition;
}

void ShooterSubsystem::Shoot(double shoot_vel, double shoot_angle)
{
  double shoot_position =
      (ShooterConstants::PitchMaxAngle - shoot_angle) * ShooterConstants::PitchDisplacementPerDegree;
  double shoot_wheel_velocity = shoot_vel / (2 * M_PI * ShooterConstants::ShootWheelRadius) /
                                ShooterConstants::SpeedConversionEfficiency /
                                shooter_left_.Getdata().gearRatio;  // 转换为转每秒

  if (shoot_angle > ShooterConstants::PitchMaxAngle || shoot_angle < ShooterConstants::PitchMinAngle)
  {
    frc::SmartDashboard::PutString("Shooter/Shoot Angle:", "ERROR:Shoot angle out of range");
    return;
  }

  SetPitchPosition(shoot_position);
  SetShootVelocity(shoot_wheel_velocity);
}

void ShooterSubsystem::Stop()
{
  SetShootVelocity(0.);
}