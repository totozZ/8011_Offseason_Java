#pragma once
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <wpinet/WebServer.h>

#include "subsystems/CommandSwerveDrivetrain.h"

namespace subsystems {
class ClientSubsystem : public frc2::SubsystemBase {
 public:
  explicit ClientSubsystem(subsystems::CommandSwerveDrivetrain* driveSubsystem);
  void Periodic() override;

  // frc::Pose2d GetTargetPose() { return targetPose; }

  // int GetLevel() { return level; }

  // void PubIsCompleted() { is_completedPub.Set(true); };

  // void PubRobotInit(bool init) {
  //   m_table->GetBooleanTopic("init").Publish().Set(init);
  // }

  // 重制赛 (REBUILT) 比赛回合和射击权定义
  enum class MatchPhase {
    kAuto,        // 自动阶段 (计时器 0:20 – 0:00，均激活)
    kTransition,  // 过渡阶段 (计时器 2:20 – 2:10，均激活)
    kSwitch1,     // 切换时段1 (计时器 2:10 – 1:45，单方激活)
    kSwitch2,     // 切换时段2 (计时器 1:45 – 1:20，单方激活)
    kSwitch3,     // 切换时段3 (计时器 1:20 – 0:55，单方激活)
    kSwitch4,     // 切换时段4 (计时器 0:55 – 0:30，单方激活)
    kEndgame,     // 最​​终阶段 (计时器 0:30 – 0:00，均激活)
    kUnknown
  };

 private:
  void UpdateMatchPhaseCountdown();

  std::shared_ptr<nt::NetworkTable> m_table;
  frc::SendableChooser<bool> m_firstAttackerChooser;
  frc::Field2d m_field;
  bool is_our_turn_first =
      true;  // Teleop切换阶段中，我方是否先激活 (由Auto分数决定)

  // nt::DoubleArraySubscriber targetPoseSub;
  // nt::IntegerSubscriber levelSub;
  // nt::BooleanPublisher is_completedPub;
  // frc::Pose2d targetPose;
  // int level;
  // bool is_completed;

  subsystems::CommandSwerveDrivetrain* m_drivesubsystem;

  // void getLevel() {
  //   auto levelData = levelSub.Get();
  //   level = levelData;
  // };

  // void getTargetPose() {
  //   auto poseData = targetPoseSub.Get();
  //   if (poseData.size() >= 3) {
  //     targetPose =
  //         frc::Pose2d(units::meter_t{poseData[0]},
  //         units::meter_t{poseData[1]},
  //                     frc::Rotation2d(units::degree_t{poseData[2]}));
  //   }
  // }
};
}  // namespace subsystems