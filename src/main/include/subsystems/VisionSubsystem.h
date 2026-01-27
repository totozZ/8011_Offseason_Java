// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/GenericEntry.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include "frc8011/AprilTags.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "LimelightHelpers.h"
#include "subsystems/LEDSubsystem.h"
namespace subsystems {
    class CommandSwerveDrivetrain;
    class GPDetection;
} // 前向声明

namespace subsystems{

  struct Arpil_Tag{
    frc::Pose2d tag_pos = frc::Pose2d(0_m, 0_m, frc::Rotation2d(90_deg)); // tag的全局位置
    float tag_ID; // AprilTag的识别ID
  };
class VisionSubsystem : public frc2::SubsystemBase {
 public:

    explicit VisionSubsystem(CommandSwerveDrivetrain* drivetrain_,
                             LEDSubsystem* ledsub_,
                             GPDetection* gpdetection_);

  /**
   * Example command factory method.
   */
  frc2::CommandPtr VisionMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool VisionCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;




 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // 依赖的子系统
    CommandSwerveDrivetrain* drivetrain_;
    LEDSubsystem* ledsub_;
    GPDetection* gpdetection_;

  // 当前机器人角速度
  double currentAngularVelocity_ = 0.0;

  // Limelight 位姿数据
  LimelightHelpers::PoseEstimate mt1_right_pose_;
  LimelightHelpers::PoseEstimate mt1_left_pose_;
  LimelightHelpers::PoseEstimate mt2_right_pose_;
  LimelightHelpers::PoseEstimate mt2_left_pose_;

  int mt_mode_ = 2; // 默认使用 mt2
  int vision_mode_ = 1; // 0视觉不更新，1更新mt2, 2更新混合
  double swiitch_distance_ = 0.8; // 1米，距离阈值，低于此距离使用混合模式

   // IMU 模式
    enum class LimelightIMUMode {
        ExternalIMU = 0,
        SeedingMode = 1,
        InternalIMU = 2,
        FusedIMU = 4
    };

    LimelightIMUMode currentIMUMode = LimelightIMUMode::ExternalIMU;
  
  std::string limelight_left_name_ = "limelight-left"; // Limelight 的名称
    
  // 信任度阈值常量
  const double kMaxAngularVelocity_ = 360.0; // 超过 360度/秒 则不信任视觉
  const double kMaxTrustDistance_ = 4.0;     // 超过 4米 则降低信任度

  /**
   * @brief 设置limelight工作模式，一共有四种，具体见LimelightIMUMode枚举
   * 
   */
  void SetLimelightIMUMode(std::string, LimelightIMUMode mode);

  /**
   * @brief 判断mt1和mt2传来的poseestimate是否应该被拒绝，检查是否有tag、tag距离是、机器人是否高速旋转、模糊度
   * 
   */
  bool ShouldRejectMetatagPose(const LimelightHelpers::PoseEstimate &poseEstimate);

  /**
   * @brief 根据距离更新视觉模式，使用mt2还是使用混合mt
   * 
   */
  void UpdateVisionMode();

  // 全场定位相关变量和方法
  /**
   * @brief 主处理函数，读取 Limelight 数据并更新里程计
   */
  void LimelightMeasurement();

  /**
   * @brief 更新当前的角速度，用于判断是否处于快速旋转中
   */
  void UpdateAngularVelocity();

  /**
   * @brief 根据距离和标签数量动态计算信任度（标准差）
   * @param distance 到 Tag 的距离
   * @param tag_count 看到的 Tag 数量
   * @return x, y, theta 的标准差数组
   */
  wpi::array<double, 3> CalculateStdDevs(double distance, int tag_count);



  /* limelight */
    AprilTags apriltags;

    frc::Pose2d tag_pos; // tag本身的全局位姿
    frc::Pose2d Vision_pos; // 处理limelight传来的相][\对姿态
    frc::Pose2d Offset_pos = frc::Pose2d(0.433_m, 0_m, frc::Rotation2d(0_deg));
    frc::Pose2d Target_pos_left; // 处理limelight传来的目标姿态
    frc::Pose2d Target_pos_right; // 处理limelight传来的目标姿态
    frc::Pose2d Target_pos_middle; // 处理limelight传来的目标姿态
    
    units::meter_t distance = 0.33_m; // 底盘中心离最外侧bumper垂直距离
    units::meter_t balldistance = 0.180_m; // 左边0.24,右边
    units::meter_t balldistance_right = 0.165_m; // 右边0.19
    units::meter_t balldistance_middle = 0.0075_m; //


    frc::Pose2d Chassis_pos;
    frc::Pose2d Chassis_pos2;


    void Cal_Vision_pos(); 
    void Get_tagpos();
    
    frc::Pose2d CalTarget_posleft();
    frc::Pose2d CalTarget_posright();



  /* 触摸屏驾驶战的数据传输topic */
  nt::NetworkTableEntry  Btn_entry;

  // 触摸屏的传输结果
  double com_pos[3] = {0.0, 0.0, 0.0}; // 触摸屏传输的目标全局位置
  double button_id = 0.0; // 触摸屏传输的标志位

  public:

  frc::Pose2d Gettag_pos() {
    return tag_pos;
  }
 

    frc::Pose2d GetChassis_pos() {
      return Chassis_pos;
    }

    frc::Pose2d GetChassis_pos2() {
      return Chassis_pos2;
    }

    frc::Pose2d GetVision_pos() {
      return Vision_pos;

    }

units::time::second_t Getlatency() {
  return apriltags.Gettimestamp();
}

    double Gettag_version() {
      frc::SmartDashboard::PutNumber("aprilGetFlag",apriltags.GetFlag());
      return apriltags.GetFlag();
    }

    

    frc::Pose2d GetTarget_posleft() {
      return Target_pos_left;
    }
    frc::Pose2d GetTarget_posright() {
      return Target_pos_right;
    }

    frc::Pose2d GetTarget_posmiddle() {
      return Target_pos_middle;
    }

    void Cal_Vision_poswithoutangle(frc::Pose2d _currentpose);

    void LED_control();
};
}
