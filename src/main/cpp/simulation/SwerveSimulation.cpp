#include "simulation/SwerveSimulation.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <cmath>

using namespace simulation;

SwerveSimulation::SwerveSimulation(
    const frc::SwerveDriveKinematics<4> &kinematics,
    const std::array<frc::Translation2d, 4> &modulePositions)
    : m_kinematics(kinematics),
      m_modulePositions(modulePositions),
      m_pose(8.27_m, 4.1_m, 0_deg), // 场地中心
      m_lastTime(frc::Timer::GetFPGATimestamp())
{
  InitializeNetworkTables();

  // 初始化模块状态
  for (auto &state : m_moduleStates)
  {
    state = frc::SwerveModuleState{0_mps, 0_deg};
  }

  for (auto &position : m_modulePositions_sim)
  {
    position = frc::SwerveModulePosition{0_m, 0_deg};
  }

  std::cout << "SwerveSimulation initialized" << std::endl;
}

void SwerveSimulation::InitializeNetworkTables()
{
  m_table = nt::NetworkTableInstance::GetDefault().GetTable("Simulation/SwerveDrive");
  m_posePub = m_table->GetStructTopic<frc::Pose2d>("Pose").Publish();
  m_moduleStatesPub = m_table->GetStructArrayTopic<frc::SwerveModuleState>("ModuleStates").Publish();
  m_modulePositionsPub = m_table->GetStructArrayTopic<frc::SwerveModulePosition>("ModulePositions").Publish();
  m_chassisSpeedsPub = m_table->GetStructTopic<frc::ChassisSpeeds>("ChassisSpeeds").Publish();
  m_gyroAnglePub = m_table->GetDoubleTopic("GyroAngle").Publish();
}

void SwerveSimulation::UpdateSimulation()
{
  units::second_t currentTime = frc::Timer::GetFPGATimestamp();
  double dt = (currentTime - m_lastTime).value();

  if (dt <= 0.0)
    return;

  // 更新模块仿真
  UpdateModuleSimulation(dt);

  // 更新里程计
  UpdateOdometry(dt);

  // 发布遥测数据
  PublishTelemetry();

  // 更新仿真管理器中的机器人位置
  SimulationManager::GetInstance().UpdateRobotPose(m_pose);

  m_lastTime = currentTime;
}

void SwerveSimulation::UpdateModuleSimulation(double dt)
{
  for (size_t i = 0; i < 4; ++i)
  {
    m_moduleSims[i].Update(m_moduleStates[i], dt);
    m_moduleStates[i] = m_moduleSims[i].GetState();
    m_modulePositions_sim[i] = m_moduleSims[i].GetPosition();
  }
}

void SwerveSimulation::UpdateOdometry(double dt)
{
  // 使用运动学计算底盘速度
  m_chassisSpeeds = m_kinematics.ToChassisSpeeds(m_moduleStates);

  // 更新机器人位置
  frc::Transform2d transform{
      m_chassisSpeeds.vx * units::second_t{dt},
      m_chassisSpeeds.vy * units::second_t{dt},
      frc::Rotation2d{m_chassisSpeeds.omega * units::second_t{dt}}};

  m_pose = m_pose.TransformBy(transform);
}

void SwerveSimulation::PublishTelemetry()
{
  m_posePub.Set(m_pose);
  m_moduleStatesPub.Set(m_moduleStates);
  m_modulePositionsPub.Set(m_modulePositions_sim);
  m_chassisSpeedsPub.Set(m_chassisSpeeds);
  m_gyroAnglePub.Set(m_pose.Rotation().Degrees().value());
}

frc::Pose3d SwerveSimulation::GetPose() const
{
  return frc::Pose3d{m_pose};
}

void SwerveSimulation::SetModuleStates(const std::array<frc::SwerveModuleState, 4> &states)
{
  m_moduleStates = states;
}

std::array<frc::SwerveModuleState, 4> SwerveSimulation::GetModuleStates() const
{
  return m_moduleStates;
}

std::array<frc::SwerveModulePosition, 4> SwerveSimulation::GetModulePositions() const
{
  return m_modulePositions_sim;
}

frc::ChassisSpeeds SwerveSimulation::GetChassisSpeeds() const
{
  return m_chassisSpeeds;
}

void SwerveSimulation::ResetPose(const frc::Pose2d &pose)
{
  m_pose = pose;
}

void SwerveSimulation::SetGyroAngle(const frc::Rotation2d &angle)
{
  m_pose = frc::Pose2d{m_pose.Translation(), angle};
}

// ModuleSimulation实现
void SwerveSimulation::ModuleSimulation::Update(const frc::SwerveModuleState &desiredState, double dt)
{
  // 简单的一阶系统仿真
  double desiredDriveVel = desiredState.speed.value() / kWheelRadius; // convert m/s to rad/s
  double desiredSteerPos = desiredState.angle.Radians().value();

  // 驱动电机仿真（速度控制）
  double driveError = desiredDriveVel - driveVelocity;
  driveVelocity += driveError * dt / kDriveTimeConstant;
  drivePosition += driveVelocity * dt;

  // 转向电机仿真（位置控制）
  double steerError = desiredSteerPos - steerPosition;

  // 处理角度环绕
  while (steerError > 3.14159265359)
    steerError -= 2 * 3.14159265359;
  while (steerError < -3.14159265359)
    steerError += 2 * 3.14159265359;

  steerVelocity = steerError / kSteerTimeConstant;
  steerPosition += steerVelocity * dt;

  // 保持转向位置在[-π, π]范围内
  while (steerPosition > 3.14159265359)
    steerPosition -= 2 * 3.14159265359;
  while (steerPosition < -3.14159265359)
    steerPosition += 2 * 3.14159265359;
}

frc::SwerveModuleState SwerveSimulation::ModuleSimulation::GetState() const
{
  return frc::SwerveModuleState{
      units::meters_per_second_t{driveVelocity * kWheelRadius},
      frc::Rotation2d{units::radian_t{steerPosition}}};
}

frc::SwerveModulePosition SwerveSimulation::ModuleSimulation::GetPosition() const
{
  return frc::SwerveModulePosition{
      units::meter_t{drivePosition * kWheelRadius},
      frc::Rotation2d{units::radian_t{steerPosition}}};
}
