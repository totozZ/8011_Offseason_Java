#include "simulation/SimulationManager.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace simulation;

SimulationManager &SimulationManager::GetInstance()
{
  static SimulationManager instance;
  return instance;
}

void SimulationManager::Initialize()
{
  if (!IsSimulation())
    return;

  std::cout << "Initializing Simulation Manager..." << std::endl;

  // 初始化NetworkTables
  m_simTable = nt::NetworkTableInstance::GetDefault().GetTable("Simulation");
  m_robotPosePub = m_simTable->GetStructTopic<frc::Pose2d>("RobotPose").Publish();
  m_timestampPub = m_simTable->GetDoubleTopic("Timestamp").Publish();
  m_enabledPub = m_simTable->GetBooleanTopic("Enabled").Publish();

  // 初始化场地
  frc::SmartDashboard::PutData("Simulation Field", &m_field);

  // 设置初始状态
  m_robotPose = frc::Pose2d{8.27_m, 4.1_m, 0_deg}; // 场地中心
  m_lastUpdateTime = frc::Timer::GetFPGATimestamp();

  std::cout << "Simulation Manager initialized successfully" << std::endl;
}

void SimulationManager::Periodic()
{
  if (!IsSimulation())
    return;

  units::second_t currentTime = frc::Timer::GetFPGATimestamp();
  if (currentTime - m_lastUpdateTime < kUpdatePeriod)
    return;

  // 执行所有更新回调
  for (auto &callback : m_updateCallbacks)
  {
    callback();
  }

  // 发布到NetworkTables
  PublishToNetworkTables();

  // 更新场地显示
  UpdateField();

  m_lastUpdateTime = currentTime;
}

void SimulationManager::UpdateRobotPose(const frc::Pose2d &pose)
{
  m_robotPose = pose;
}

void SimulationManager::AddUpdateCallback(std::function<void()> callback)
{
  m_updateCallbacks.push_back(callback);
}

void SimulationManager::PublishToNetworkTables()
{
  m_robotPosePub.Set(m_robotPose);
  m_timestampPub.Set(frc::Timer::GetFPGATimestamp().value());
  m_enabledPub.Set(true);
}

void SimulationManager::UpdateField()
{
  m_field.SetRobotPose(m_robotPose);
}