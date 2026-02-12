#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/angle.h>

#include <cmath>
#include <iostream>
#include <string>

#include "frc8011/GPDetection.h"
using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder() {
  // 设置机器人的初始朝向，模拟赛场的不同颜色，机器人正方向与操作手平�?
  auto config = pathplanner::RobotConfig::fromGUISettings();
  pathplanner::AutoBuilder::configure(
      // Supplier of current robot pose

      [this] { return GetState().Pose; },
      //[this] { return VR8011.getPose2d(); },
      // Consumer for seeding pose against auto
      [this](frc::Pose2d const& pose) { return ResetPose(pose); },
      //[this](frc::Pose2d const &pose) { return VR8011.zeroAllPosition(); },
      // Supplier of current robot speeds[\]
      [this] { return GetState().Speeds; },
      // Consumer of ChassisSpeeds and feedforwards to drive the robot
      [this](frc::ChassisSpeeds const& speeds,
             pathplanner::DriveFeedforwards const& feedforwards) {
        return SetControl(
            m_pathApplyRobotSpeeds
                .WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 20_ms))
                .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                .WithWheelForceFeedforwardsY(
                    feedforwards.robotRelativeForcesY));
      },
      std::make_shared<pathplanner::PPHolonomicDriveController>(
          // PID constants for translation
          pathplanner::PIDConstants{10.0, 0.0, 0.0},
          // PID constants for rotation
          pathplanner::PIDConstants{7.0, 0.0, 0.0}),

      std::move(config),

      // Assume the path needs to be flipped for Red vs Blue, this is normally
      // the case
      [] {
        auto const alliance = frc::DriverStation::GetAlliance().value_or(
            frc::DriverStation::Alliance::kBlue);
        return alliance == frc::DriverStation::Alliance::kRed;
      },

      this  // Subsystem for requirements
  );
}

void CommandSwerveDrivetrain::Periodic() {
  /*
   * Periodically try to apply the operator perspective.
   * If we haven't applied the operator perspective before, then we should apply
   * it regardless of DS state. This allows us to correct the perspective in
   * case the robot code restarts mid-match. Otherwise, only check and apply the
   * operator perspective if the DS is disabled. This ensures driving behavior
   * doesn't change until an explicit disable event occurs during testing.
   */
  if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    auto const allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor) {
      SetOperatorPerspectiveForward(*allianceColor ==
                                            frc::DriverStation::Alliance::kRed
                                        ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }
  }

  // frc::SmartDashboard::PutNumber("NEWYAW",ChassisPosedFromAprilTag().Rotation().Radians());
  // frc::Pose2d currentPose = GetState().Pose;
  // frc::Pose2d currentPose = VR8011.getPose2d();

  currentPose = GetState().Pose;

  // 打印当前里程计位�?
  frc::SmartDashboard::PutNumberArray(
      "CurrentPose_code",
      std::vector<double>{GetState().Pose.Translation().X().value(),
                          GetState().Pose.Translation().Y().value(),
                          GetState().Pose.Rotation().Degrees().value()});

  auto modules = GetModules();
  for (size_t i = 0; i < modules.size(); ++i) {
    const std::string modulePrefix = "Swerve/Module" + std::to_string(i);
    frc::SmartDashboard::PutNumber(
        modulePrefix + "/Drive/SupplyCurrent",
        modules[i]->GetDriveMotor().GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber(
        modulePrefix + "/Drive/TorqueCurrent",
        modules[i]->GetDriveMotor().GetTorqueCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber(
        modulePrefix + "/Steer/SupplyCurrent",
        modules[i]->GetSteerMotor().GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber(
        modulePrefix + "/Steer/TorqueCurrent",
        modules[i]->GetSteerMotor().GetTorqueCurrent().GetValue().value());
  }

  // gene_path = GeneratePath(frc::Pose2d{1_m, 1_m, frc::Rotation2d{90_deg}});

  frc::SmartDashboard::PutNumber("eventflag", eventflag);
};

void CommandSwerveDrivetrain::StartSimThread() {
  m_lastSimTime = utils::GetCurrentTime();
  m_simNotifier = std::make_unique<frc::Notifier>([this] {
    units::second_t const currentTime = utils::GetCurrentTime();
    auto const deltaTime = currentTime - m_lastSimTime;
    m_lastSimTime = currentTime;

    /* use the measured time delta, get battery voltage from WPILib */
    UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
  });
  m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::GeneratePath(
    frc::Pose2d targetPose) {
  static double time2 = 0;
  static double totalTime = 0;
  // 获取当前机器人的位姿
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  // 测量路径生成时间
  auto pathStartTime = frc::Timer::GetFPGATimestamp();

  //     // 获取当前和目标点的底盘旋转角�?
  // frc::Rotation2d current_angle = currentPose.Rotation();
  // frc::Rotation2d target_angle = targetPose.Rotation();
  // frc::Pose2d startpoint{
  //     // frc::Translation2d{0.5_m, 0.5_m},
  //     currentPose.Translation(),
  //     start_heading}; // 起始�?

  // frc::Pose2d targetpoint{
  //     targetPose.Translation(),
  //     end_heading}; // 目标�?

  // 使用距离检查目标和当前是否过近
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_距离", 1);
    return nullptr;
  }

  // 检查目标点是否构造无效的旋转目标
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_旋转", 1);
    return nullptr;
  }

  // 获取当前和目标点的底盘旋转角�?
  frc::Rotation2d target_angle = targetPose.Rotation();

  // double vx = GetState().Speeds.vx();
  // double vy = GetState().Speeds.vy();
  // double v = sqrt(vx * vx + vy * vy);
  // frc::Rotation2d start_heading = frc::Rotation2d{units::radian_t{atan2(vy,
  // vx)}}; frc::Rotation2d end_heading = targetPose.Translation().Angle();

  // 取当前位姿和目标位姿的中点作为路径的第一个waypoint,注意此时waypoints第三个值rotation并非为底盘旋转角度而是机器人此时的行进朝向
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 两点之间相对于x轴的角度
  frc::Rotation2d targetpoint_heading =
      target_angle;  // 最后点的行进朝向应为目标角�?

  // frc::Rotation2d waypoint_heading = temp_heading.Degrees() > 0_deg ?
  // (-180_deg + temp_heading.Degrees()) : (180_deg + temp_heading.Degrees());
  // // 计算直线行驶时的行进朝向

  // frc::SmartDashboard::PutNumber("temp_heading",
  // temp_heading.Degrees().value());
  // frc::SmartDashboard::PutNumber("waypoint_heading",
  // waypoint_heading.Degrees().value());

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 起始�?

  frc::Pose2d waypoint1{
      (currentPose.Translation() + targetPose.Translation()) / 2.0,
      waypoint_heading};  // 两点之间的中�?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          targetpoint_heading};  // 目标�?

  // targetpoint_heading 是指�?Reef
  // 的方向，退�?.6m创建一个waypoint,配合视觉模式2
  frc::Pose2d Reefpoint{
      targetPose.Translation() -
          frc::Translation2d{targetpoint_heading.Cos() * 0.6_m,
                             targetpoint_heading.Sin() * 0.6_m},
      targetpoint_heading};

  // 创建动态路径的waypoints
  std::vector<frc::Pose2d> poses = {startpoint,
                                    // Reefpoint,
                                    // waypoint1,
                                    targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 设置路径约束（可以根据需要调整最大速度和加速度�?/2
  PathConstraints constraints(2.5_mps, 2_mps_sq, 540_deg_per_s,
                              980_deg_per_s_sq);

  // PathConstraints constraints(2.8_mps, 4.0_mps_sq, 640_deg_per_s,
  // 1180_deg_per_s_sq); frc::SmartDashboard::PutNumber("V", v);
  // frc::SmartDashboard::PutNumber("start_heading",
  // start_heading.Degrees().value()); PathConstraints constraints(0.7_mps,
  // 0.7_mps_sq, 540_deg_per_s, 980_deg_per_s_sq);

  // 创建路径
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints,
      // IdealStartingState(0_mps, current_angle), //
      // 空的起始状态，对于动态生成的路径，我们不需要理想的起始状�?
      std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 目标状态，这里设置为给定目标速度和角�?
  );

  // 防止路径在正确的坐标下被翻转
  path->preventFlipping = true;

  // 计算路径生成耗时
  auto pathEndTime = frc::Timer::GetFPGATimestamp();
  auto pathGenerationTime = (pathEndTime - pathStartTime).value() * 1000;

  time2++;
  totalTime += pathGenerationTime;
  double averageTime = totalTime / time2;
  frc::SmartDashboard::PutNumber("create path time", pathGenerationTime);
  frc::SmartDashboard::PutNumber("average path time", averageTime);

  // 记录数据
  frc::SmartDashboard::PutNumberArray(
      "startpoint",
      std::vector<double>{startpoint.Translation().X().value(),
                          startpoint.Translation().Y().value(),
                          startpoint.Rotation().Degrees().value()});
  // frc::SmartDashboard::PutNumberArray("waypoint",
  // std::vector<double>{waypoint1.Translation().X().value(),
  // waypoint1.Translation().Y().value(), waypoint_heading.Degrees().value()});
  frc::SmartDashboard::PutNumberArray(
      "endpoint", std::vector<double>{targetPose.Translation().X().value(),
                                      targetPose.Translation().Y().value(),
                                      waypoint_heading.Degrees().value()});

  frc::SmartDashboard::PutNumber("create success", time2);

  return path;
}

std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::AutoGeneratePath(
    frc::Pose2d targetPose, double _maxspeed, double _maxacc) {
  static double time2 = 1;

  // 获取当前机器人的位姿
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  //     // 获取当前和目标点的底盘旋转角�?
  // frc::Rotation2d current_angle = currentPose.Rotation();
  // frc::Rotation2d target_angle = targetPose.Rotation();
  // frc::Pose2d startpoint{
  //     // frc::Translation2d{0.5_m, 0.5_m},
  //     currentPose.Translation(),
  //     start_heading}; // 起始�?

  // frc::Pose2d targetpoint{
  //     targetPose.Translation(),
  //     end_heading}; // 目标�?

  // 使用距离检查目标和当前是否过近
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_距离", 1);
    return nullptr;
  }

  // 检查目标点是否构造无效的旋转目标
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_旋转", 1);
    return nullptr;
  }

  // frc::Rotation2d current_angle = currentPose.Rotation();
  frc::Rotation2d target_angle = targetPose.Rotation();

  // double vx = GetState().Speeds.vx();
  // double vy = GetState().Speeds.vy();
  // double v = sqrt(vx * vx + vy * vy);
  // frc::Rotation2d start_heading = frc::Rotation2d{units::radian_t{atan2(vy,
  // vx)}}; frc::Rotation2d end_heading = targetPose.Translation().Angle();

  // 取当前位姿和目标位姿的中点作为路径的第一个waypoint,注意此时waypoints第三个值rotation并非为底盘旋转角度而是机器人此时的行进朝向
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 两点之间相对于x轴的角度
  // frc::Rotation2d waypoint_heading = temp_heading.Degrees() > 0_deg ?
  // (-180_deg + temp_heading.Degrees()) : (180_deg + temp_heading.Degrees());
  // // 计算直线行驶时的行进朝向

  // frc::SmartDashboard::PutNumber("temp_heading",
  // temp_heading.Degrees().value());
  // frc::SmartDashboard::PutNumber("waypoint_heading",
  // waypoint_heading.Degrees().value());

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 起始�?

  frc::Pose2d waypoint1{
      (currentPose.Translation() + targetPose.Translation()) / 2.0,
      waypoint_heading};  // 两点之间的中�?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          waypoint_heading};  // 目标�?

  // 创建动态路径的waypoints
  std::vector<frc::Pose2d> poses = {startpoint, waypoint1, targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 设置路径约束（可以根据需要调整最大速度和加速度�?/2
  PathConstraints constraints(_maxspeed * 1_mps, _maxacc * 1_mps_sq,
                              540_deg_per_s, 980_deg_per_s_sq);
  // frc::SmartDashboard::PutNumber("V", v);
  // frc::SmartDashboard::PutNumber("start_heading",
  // start_heading.Degrees().value()); PathConstraints constraints(0.7_mps,
  // 0.7_mps_sq, 540_deg_per_s, 980_deg_per_s_sq); 创建路径
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints,
      // IdealStartingState(0_mps, current_angle), //
      // 空的起始状态，对于动态生成的路径，我们不需要理想的起始状�?
      std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 目标状态，这里设置为给定目标速度和角�?
  );
  // frc::SmartDashboard::PutNumberArray("path.poses",
  // path.get()->bezierFromPoses[1]); 防止路径在正确的坐标下被翻转
  path->preventFlipping = true;

  // path123 = *path;

  // 记录数据
  frc::SmartDashboard::PutNumberArray(
      "startpoint",
      std::vector<double>{startpoint.Translation().X().value(),
                          startpoint.Translation().Y().value(),
                          startpoint.Rotation().Degrees().value()});
  // frc::SmartDashboard::PutNumberArray("waypoint",
  // std::vector<double>{waypoint1.Translation().X().value(),
  // waypoint1.Translation().Y().value(), waypoint_heading.Degrees().value()});
  frc::SmartDashboard::PutNumberArray(
      "endpoint", std::vector<double>{targetPose.Translation().X().value(),
                                      targetPose.Translation().Y().value(),
                                      waypoint_heading.Degrees().value()});

  frc::SmartDashboard::PutNumber("create success", time2++);

  return path;
}
/**
 * @brief 从多个目标点生成路径
 *
 * @param targetPoses Pose2d数组,其中:
 *   - 前N-1个点的Rotation表示**行进方向(heading)**
 *   - 最�?个点的Rotation表示**最终底盘朝�?target rotation)**
 *
 * @return 生成的路�?失败返回nullptr
 *
 * @example
 * std::vector<frc::Pose2d> waypoints = {
 *     {2_m, 3_m, 45_deg},   // �?5°方向行进
 *     {4_m, 5_m, 90_deg},   // �?0°方向行进
 *     {6_m, 7_m, 180_deg}   // 到达后车头朝180°
 * };
 */
std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::GeneratePath(
    std::vector<frc::Pose2d> const& targetPoses) {
  if (targetPoses.empty()) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_无目标点", 1);
    return nullptr;
  }

  frc::Pose2d currentPose = GetState().Pose;

  // 验证第一个点距离
  double deltaX = targetPoses[0].Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPoses[0].Translation().Y().value() -
                  currentPose.Translation().Y().value();
  if (std::sqrt(deltaX * deltaX + deltaY * deltaY) < 0.05) {
    return nullptr;
  }

  // �?构建路径点列�?
  std::vector<frc::Pose2d> poses;

  // 起点: 朝向第一个目�?
  frc::Rotation2d initial_heading =
      (targetPoses[0].Translation() - currentPose.Translation()).Angle();
  poses.push_back({currentPose.Translation(), initial_heading});

  // �?关键修改: 直接使用用户传入的Rotation作为heading
  for (size_t i = 0; i < targetPoses.size(); ++i) {
    poses.push_back(targetPoses[i]);
  }

  // �?创建路径
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 最后一个点的Rotation既是heading也是target_rotation
  frc::Rotation2d final_target_rotation = targetPoses.back().Rotation();

  // PathConstraints constraints(2.5_mps, 4_mps_sq, 540_deg_per_s,
  // 690_deg_per_s_sq);
  PathConstraints constraints(1.8_mps, 1.8_mps_sq, 640_deg_per_s,
                              980_deg_per_s_sq);

  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints, std::nullopt,
      GoalEndState(0_mps, final_target_rotation)  // 使用最后一个点的Rotation
  );

  path->preventFlipping = true;

  return path;
}

frc2::CommandPtr CommandSwerveDrivetrain::followPathCommand(
    frc::Pose2d targetPose) {
  static double failtime = 0;
  auto goalPath = GeneratePath(targetPose);
  if (goalPath == nullptr) {
    return frc2::cmd::RunOnce(
        [] { frc::SmartDashboard::PutNumber("failtime ", failtime++); });
  } else {
    // return frc2::cmd::RunOnce([]{frc::SmartDashboard::PutNumber("successtime
    // ", failtime++);});
    return AutoBuilder::followPath(goalPath);
  }
  // return frc2::cmd::RunOnce([]{frc::SmartDashboard::PutNumber("useful ",
  // followtime++);});
}

frc2::CommandPtr CommandSwerveDrivetrain::AutofollowPathCommand(
    frc::Pose2d targetPose, double _maxspeed, double _maxacc) {
  static double failtime = 0;
  auto goalPath = AutoGeneratePath(targetPose, _maxspeed, _maxacc);
  if (goalPath == nullptr) {
    return frc2::cmd::RunOnce(
        [] { frc::SmartDashboard::PutNumber("failtime ", failtime++); });
  } else {
    return AutoBuilder::followPath(goalPath);
  }
}

// 点版�?- 新增
frc2::CommandPtr CommandSwerveDrivetrain::followPathCommand(
    std::vector<frc::Pose2d> const& targetPoses) {
  static double failtime = 0;
  static double successtime = 0;

  // 调用多点版本�?GeneratePath
  auto goalPath = GeneratePath(targetPoses);

  if (goalPath == nullptr) {
    failtime++;
    double currentFailtime = failtime;
    return frc2::cmd::RunOnce([currentFailtime] {
      frc::SmartDashboard::PutNumber("MultiPath_FailCount", currentFailtime);
    });
  } else {
    successtime++;
    frc::SmartDashboard::PutNumber("MultiPath_SuccessCount", successtime);
    frc::SmartDashboard::PutNumber("MultiPath_TargetCount", targetPoses.size());

    return AutoBuilder::followPath(goalPath);
  }
}

std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::GeneratePath(
    frc::Pose2d targetPose, frc2::CommandPtr eventCommand) {
  // 获取当前机器人的位姿
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  // 使用距离检查目标和当前是否过近
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_距离", 1);
    return nullptr;
  }

  // 检查目标点是否构造无效的旋转目标
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("轨迹函数创建失败_旋转", 1);
    return nullptr;
  }

  // 获取当前和目标点的底盘旋转角�?
  frc::Rotation2d target_angle = targetPose.Rotation();

  // 取当前位姿和目标位姿的中点作为路径的第一个waypoint,注意此时waypoints第三个值rotation并非为底盘旋转角度而是机器人此时的行进朝向
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 两点之间相对于x轴的角度
  frc::Rotation2d targetpoint_heading =
      target_angle;  // 最后点的行进朝向应为目标角�?

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 起始�?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          targetpoint_heading};  // 目标�?
  // targetpoint_heading 是指�?Reef
  // 的方向，退�?.6m创建一个waypoint,配合视觉模式2
  frc::Pose2d Reefpoint{
      targetPose.Translation() -
          frc::Translation2d{targetpoint_heading.Cos() * 0.75_m,
                             targetpoint_heading.Sin() * 0.6_m},
      targetpoint_heading};

  // 创建动态路径的waypoints
  std::vector<frc::Pose2d> poses = {startpoint, Reefpoint,
                                    // waypoint1,
                                    targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 设置路径约束（可以根据需要调整最大速度和加速度�?/2
  PathConstraints constraints(1.8_mps, 1.8_mps_sq, 640_deg_per_s,
                              980_deg_per_s_sq);

  // 创建路径
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints, std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 目标状态，这里设置为给定目标速度和角�?
  );

  // 防止路径在正确的坐标下被翻转
  path->preventFlipping = true;

  // 创建并添�?EventMarker
  // ReefPoint 是第 2 个点 (索引�?1)，我们希望在到达 ReefPoint 时触发命�?
  path->getEventMarkers().push_back(pathplanner::EventMarker(
      "event1_trigger",                              // 触发器名�?
      1.0,                                           // 位置 (�?2 个点)
      frc2::cmd::RunOnce([this] { eventflag = 1; })  // 传入command
      ));

  // 创建并添�?
  // RotationTarget,在reefpoint即垂直珊瑚礁，使第二段仅调节向前的量，并且搭配混合视觉模式纠正机器人旋转方向
  path->getRotationTargets().push_back(pathplanner::RotationTarget(
      1.0,          // 位置 (�?2 个点)
      target_angle  // 目标旋转角度
      ));

  return path;
}

// DriveAiming实现

frc2::CommandPtr CommandSwerveDrivetrain::DriveAimingCommand(
    std::function<double()> xSupplier, std::function<double()> ySupplier,
    units::meters_per_second_t maxSpeed) {
  return frc2::cmd::Run(
             [this, xSupplier, ySupplier, maxSpeed] {
               const auto currentPose = GetState().Pose;
               const double currentYawDeg =
                   currentPose.Rotation().Degrees().value();
               const auto targetAngle = CalculateTargetAngleToHub();
               const double targetYawDeg = targetAngle.Degrees().value();
               SetControl(
                   m_driveAimingRequest.WithVelocityX(xSupplier() * maxSpeed)
                       .WithVelocityY(ySupplier() * maxSpeed)
                       .WithTargetDirection(targetAngle)
                       .WithMaxAbsRotationalRate(
                           DriveAimingConstants::MaxDriveAimingOmega)
                       .WithDriveRequestType(swerve::DriveRequestType::Velocity)
                       .WithSteerRequestType(
                           swerve::SteerRequestType::Position));

               frc::SmartDashboard::PutNumber("DriveAiming/TargetYawDeg",
                                              targetYawDeg);
               frc::SmartDashboard::PutNumber("DriveAiming/CurrentYawDeg",
                                              currentYawDeg);
               frc::SmartDashboard::PutNumber(
                   "DriveAiming/AngleErrorDeg",
                   NormalizeAngle(targetYawDeg - currentYawDeg));
               frc::SmartDashboard::PutNumber(
                   "DriveAiming/OmegaRadPerSec",
                   m_driveAimingRequest.HeadingController
                       .GetLastAppliedOutput());
               frc::SmartDashboard::PutBoolean(
                   "DriveAiming/OnTarget",
                   m_driveAimingRequest.HeadingController.AtSetpoint());
             },
             {this})
      .BeforeStarting([this, maxSpeed] {
        m_driveAimingRequest.WithHeadingPID(7.0, 0.0, 0.02)
            .WithRotationalDeadband(units::radians_per_second_t{0.2})
            .WithMaxAbsRotationalRate(units::radians_per_second_t{6.14})
            .WithDeadband(maxSpeed * 0.05)
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
      })
      .FinallyDo(
          [this](bool) { SetControl(swerve::requests::SwerveDriveBrake{}); });
}

frc::Rotation2d CommandSwerveDrivetrain::CalculateTargetAngleToHub() {
  auto robotPose = GetState().Pose;
  auto hubPos = GetHubPosition();

  auto hub_robot_x = hubPos.X() - robotPose.X();
  auto hub_robot_y = hubPos.Y() - robotPose.Y();

  return frc::Rotation2d{units::math::atan2(hub_robot_y, hub_robot_x)};
}

frc::Translation2d CommandSwerveDrivetrain::GetHubPosition() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return DriveAimingConstants::RedHubPosition;
  }
  return DriveAimingConstants::BlueHubPosition;
}

double CommandSwerveDrivetrain::NormalizeAngle(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}
