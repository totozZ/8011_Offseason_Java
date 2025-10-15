#pragma once

#include "simulation/SimulationManager.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <array>
#include <memory>
#include <cmath>

namespace simulation
{
  /**
   * @brief Swerve驱动仿真组件
   */
  class SwerveSimulation
  {
  public:
    /**
     * @brief 构造函数
     * @param kinematics Swerve运动学对象
     * @param modulePositions 模块位置（相对于机器人中心）
     */
    SwerveSimulation(
        const frc::SwerveDriveKinematics<4> &kinematics,
        const std::array<frc::Translation2d, 4> &modulePositions);

    /**
     * @brief 更新仿真状态
     */
    void UpdateSimulation();

    /**
     * @brief 获取当前机器人位置（3D版本，用于兼容）
     */
    frc::Pose3d GetPose() const;

    /**
     * @brief 设置期望的模块状态
     * @param states 四个模块的期望状态
     */
    void SetModuleStates(const std::array<frc::SwerveModuleState, 4> &states);

    /**
     * @brief 获取当前模块状态
     */
    std::array<frc::SwerveModuleState, 4> GetModuleStates() const;

    /**
     * @brief 获取当前模块位置
     */
    std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;

    /**
     * @brief 获取当前底盘速度
     */
    frc::ChassisSpeeds GetChassisSpeeds() const;

    /**
     * @brief 重置机器人位置
     * @param pose 新的位置
     */
    void ResetPose(const frc::Pose2d &pose);

    /**
     * @brief 获取当前2D位置
     */
    frc::Pose2d GetPose2d() const { return m_pose; }

    /**
     * @brief 设置陀螺仪角度
     * @param angle 角度
     */
    void SetGyroAngle(const frc::Rotation2d &angle);

    /**
     * @brief 获取陀螺仪角度
     */
    frc::Rotation2d GetGyroAngle() const { return m_pose.Rotation(); }

  private:
    // 运动学和物理参数
    frc::SwerveDriveKinematics<4> m_kinematics;
    std::array<frc::Translation2d, 4> m_modulePositions;

    // 仿真状态
    frc::Pose2d m_pose;
    std::array<frc::SwerveModuleState, 4> m_moduleStates;
    std::array<frc::SwerveModulePosition, 4> m_modulePositions_sim;
    frc::ChassisSpeeds m_chassisSpeeds;

    // 时间管理
    units::second_t m_lastTime;

    // NetworkTables发布器
    std::shared_ptr<nt::NetworkTable> m_table;
    nt::StructPublisher<frc::Pose2d> m_posePub;
    nt::StructArrayPublisher<frc::SwerveModuleState> m_moduleStatesPub;
    nt::StructArrayPublisher<frc::SwerveModulePosition> m_modulePositionsPub;
    nt::StructPublisher<frc::ChassisSpeeds> m_chassisSpeedsPub;
    nt::DoublePublisher m_gyroAnglePub;

    // 物理仿真参数
    static constexpr double kMaxSpeed = 4.5;                      // m/s
    static constexpr double kMaxAngularSpeed = 2 * 3.14159265359; // rad/s
    static constexpr double kWheelRadius = 0.0508;                // meters (2 inches)
    static constexpr double kDriveGearRatio = 6.48;
    static constexpr double kSteerGearRatio = 12.1;

    void InitializeNetworkTables();
    void UpdateModuleSimulation(double dt);
    void UpdateOdometry(double dt);
    void PublishTelemetry();

    /**
     * @brief 仿真单个模块的物理特性
     */
    struct ModuleSimulation
    {
      double drivePosition = 0.0; // radians
      double steerPosition = 0.0; // radians
      double driveVelocity = 0.0; // rad/s
      double steerVelocity = 0.0; // rad/s

      // 简单的一阶系统仿真
      static constexpr double kDriveTimeConstant = 0.1;  // seconds
      static constexpr double kSteerTimeConstant = 0.05; // seconds

      void Update(const frc::SwerveModuleState &desiredState, double dt);
      frc::SwerveModuleState GetState() const;
      frc::SwerveModulePosition GetPosition() const;
    };

    std::array<ModuleSimulation, 4> m_moduleSims;
  };
}
