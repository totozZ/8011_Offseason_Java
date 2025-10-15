#pragma once

#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>
#include <memory>
#include <vector>
#include <functional>

namespace simulation
{
  /**
   * @brief 简化的仿真管理器，只负责基本的仿真协调
   */
  class SimulationManager
  {
  public:
    static SimulationManager &GetInstance();

    /**
     * @brief 初始化仿真系统
     */
    void Initialize();

    /**
     * @brief 定期更新仿真状态
     */
    void Periodic();

    /**
     * @brief 更新机器人位置
     * @param pose 新的机器人位置
     */
    void UpdateRobotPose(const frc::Pose2d &pose);

    /**
     * @brief 获取当前机器人位置
     */
    frc::Pose2d GetRobotPose() const { return m_robotPose; }

    /**
     * @brief 检查是否在仿真模式
     */
    bool IsSimulation() const { return frc::RobotBase::IsSimulation(); }

    /**
     * @brief 添加仿真更新回调
     */
    void AddUpdateCallback(std::function<void()> callback);

  private:
    SimulationManager() = default;

    // NetworkTables发布器
    std::shared_ptr<nt::NetworkTable> m_simTable;
    nt::StructPublisher<frc::Pose2d> m_robotPosePub;
    nt::DoublePublisher m_timestampPub;
    nt::BooleanPublisher m_enabledPub;

    // 仿真状态
    frc::Pose2d m_robotPose;
    std::vector<std::function<void()>> m_updateCallbacks;

    // 场地可视化
    frc::Field2d m_field;

    // 时间管理
    units::second_t m_lastUpdateTime{0_s};
    static constexpr units::second_t kUpdatePeriod{20_ms}; // 50Hz更新率

    void PublishToNetworkTables();
    void UpdateField();
  };
}