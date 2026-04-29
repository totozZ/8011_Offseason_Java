#include "subsystems/CommandSwerveDrivetrain.h"

#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/angle.h>

#include <cmath>
#include <iostream>
#include <string>
#include <thread>

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder() {
  // 璁剧疆鏈哄櫒浜虹殑鍒濆鏈濆悜锛屾ā鎷熻禌鍦虹殑涓嶅悓棰滆壊锛屾満鍣ㄤ汉姝ｆ柟鍚戜笌鎿嶄綔鎵嬪钩锟?
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
          pathplanner::PIDConstants{5, 0.0, 0.0},
          // PID constants for rotation
          pathplanner::PIDConstants{8, 0.0, 0.0}),

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
  static constexpr double kPeriodicOverrunMs = 5.0;
  static int periodic_overrun_count = 0;
  static double periodic_ms_max = 0.0;
  static double last_debug_publish_s = -1.0;
  static constexpr double kDebugPublishPeriodS = 0.1;
  const double t_start_s = frc::Timer::GetFPGATimestamp().value();
  const bool publish_debug =
      (last_debug_publish_s < 0.0) ||
      ((t_start_s - last_debug_publish_s) >= kDebugPublishPeriodS);
  if (publish_debug) {
    last_debug_publish_s = t_start_s;
  }

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

  // 鎵撳嵃褰撳墠閲岀▼璁′綅锟?
  if (publish_debug) {
    frc::SmartDashboard::PutNumberArray(
        "CurrentPose_code",
        std::vector<double>{GetState().Pose.Translation().X().value(),
                            GetState().Pose.Translation().Y().value(),
                            GetState().Pose.Rotation().Degrees().value()});

    frc::SmartDashboard::PutNumber("eventflag", eventflag);
  }
  // gene_path = GeneratePath(frc::Pose2d{1_m, 1_m, frc::Rotation2d{90_deg}});

  const double periodic_ms =
      (frc::Timer::GetFPGATimestamp().value() - t_start_s) * 1000.0;
  if (periodic_ms > periodic_ms_max) {
    periodic_ms_max = periodic_ms;
  }
  if (periodic_ms > kPeriodicOverrunMs) {
    ++periodic_overrun_count;
  }
  frc::SmartDashboard::PutNumber("Perf/DrivetrainPeriodicMs", periodic_ms);
  frc::SmartDashboard::PutNumber("Perf/DrivetrainPeriodicMsMax", periodic_ms_max);
  frc::SmartDashboard::PutNumber("Perf/DrivetrainPeriodicOverrunCount",
                                 periodic_overrun_count);
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
  // 鑾峰彇褰撳墠鏈哄櫒浜虹殑浣嶅Э
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  // 娴嬮噺璺緞鐢熸垚鏃堕棿
  auto pathStartTime = frc::Timer::GetFPGATimestamp();

  //     // 鑾峰彇褰撳墠鍜岀洰鏍囩偣鐨勫簳鐩樻棆杞锟?
  // frc::Rotation2d current_angle = currentPose.Rotation();
  // frc::Rotation2d target_angle = targetPose.Rotation();
  // frc::Pose2d startpoint{
  //     // frc::Translation2d{0.5_m, 0.5_m},
  //     currentPose.Translation(),
  //     start_heading}; // 璧峰锟?

  // frc::Pose2d targetpoint{
  //     targetPose.Translation(),
  //     end_heading}; // 鐩爣锟?

  // 浣跨敤璺濈妫€鏌ョ洰鏍囧拰褰撳墠鏄惁杩囪繎
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_璺濈", 1);
    return nullptr;
  }

  // 妫€鏌ョ洰鏍囩偣鏄惁鏋勯€犳棤鏁堢殑鏃嬭浆鐩爣
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_鏃嬭浆", 1);
    return nullptr;
  }

  // 鑾峰彇褰撳墠鍜岀洰鏍囩偣鐨勫簳鐩樻棆杞锟?
  frc::Rotation2d target_angle = targetPose.Rotation();

  // double vx = GetState().Speeds.vx();
  // double vy = GetState().Speeds.vy();
  // double v = sqrt(vx * vx + vy * vy);
  // frc::Rotation2d start_heading = frc::Rotation2d{units::radian_t{atan2(vy,
  // vx)}}; frc::Rotation2d end_heading = targetPose.Translation().Angle();

  // 鍙栧綋鍓嶄綅濮垮拰鐩爣浣嶅Э鐨勪腑鐐逛綔涓鸿矾寰勭殑绗竴涓獁aypoint,娉ㄦ剰姝ゆ椂waypoints绗笁涓€紃otation骞堕潪涓哄簳鐩樻棆杞搴﹁€屾槸鏈哄櫒浜烘鏃剁殑琛岃繘鏈濆悜
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 涓ょ偣涔嬮棿鐩稿浜巟杞寸殑瑙掑害
  frc::Rotation2d targetpoint_heading =
      target_angle;  // 鏈€鍚庣偣鐨勮杩涙湞鍚戝簲涓虹洰鏍囪锟?

  // frc::Rotation2d waypoint_heading = temp_heading.Degrees() > 0_deg ?
  // (-180_deg + temp_heading.Degrees()) : (180_deg + temp_heading.Degrees());
  // // 璁＄畻鐩寸嚎琛岄┒鏃剁殑琛岃繘鏈濆悜

  // frc::SmartDashboard::PutNumber("temp_heading",
  // temp_heading.Degrees().value());
  // frc::SmartDashboard::PutNumber("waypoint_heading",
  // waypoint_heading.Degrees().value());

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 璧峰锟?

  frc::Pose2d waypoint1{
      (currentPose.Translation() + targetPose.Translation()) / 2.0,
      waypoint_heading};  // 涓ょ偣涔嬮棿鐨勪腑锟?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          waypoint_heading};  // 鐩爣锟?


  // 鍒涘缓鍔ㄦ€佽矾寰勭殑waypoints
  std::vector<frc::Pose2d> poses = {startpoint,waypoint1,
                                    targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 璁剧疆璺緞绾︽潫锛堝彲浠ユ牴鎹渶瑕佽皟鏁存渶澶ч€熷害鍜屽姞閫熷害锟?/2
  PathConstraints constraints(1.25_mps, 1_mps_sq, 270_deg_per_s,
                              240_deg_per_s_sq);

  // PathConstraints constraints(2.8_mps, 4.0_mps_sq, 640_deg_per_s,
  // 1180_deg_per_s_sq); frc::SmartDashboard::PutNumber("V", v);
  // frc::SmartDashboard::PutNumber("start_heading",
  // start_heading.Degrees().value()); PathConstraints constraints(0.7_mps,
  // 0.7_mps_sq, 540_deg_per_s, 980_deg_per_s_sq);

  // 鍒涘缓璺緞
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints,
      // IdealStartingState(0_mps, current_angle), //
      // 绌虹殑璧峰鐘舵€侊紝瀵逛簬鍔ㄦ€佺敓鎴愮殑璺緞锛屾垜浠笉闇€瑕佺悊鎯崇殑璧峰鐘讹拷?
      std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 鐩爣鐘舵€侊紝杩欓噷璁剧疆涓虹粰瀹氱洰鏍囬€熷害鍜岃锟?
  );

  // 闃叉璺緞鍦ㄦ纭殑鍧愭爣涓嬭缈昏浆
  path->preventFlipping = true;

  // 璁＄畻璺緞鐢熸垚鑰楁椂
  auto pathEndTime = frc::Timer::GetFPGATimestamp();
  auto pathGenerationTime = (pathEndTime - pathStartTime).value() * 1000;

  time2++;
  totalTime += pathGenerationTime;
  double averageTime = totalTime / time2;
  frc::SmartDashboard::PutNumber("create path time", pathGenerationTime);
  frc::SmartDashboard::PutNumber("average path time", averageTime);

  // 璁板綍鏁版嵁
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

  // 鑾峰彇褰撳墠鏈哄櫒浜虹殑浣嶅Э
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  //     // 鑾峰彇褰撳墠鍜岀洰鏍囩偣鐨勫簳鐩樻棆杞锟?
  // frc::Rotation2d current_angle = currentPose.Rotation();
  // frc::Rotation2d target_angle = targetPose.Rotation();
  // frc::Pose2d startpoint{
  //     // frc::Translation2d{0.5_m, 0.5_m},
  //     currentPose.Translation(),
  //     start_heading}; // 璧峰锟?

  // frc::Pose2d targetpoint{
  //     targetPose.Translation(),
  //     end_heading}; // 鐩爣锟?

  // 浣跨敤璺濈妫€鏌ョ洰鏍囧拰褰撳墠鏄惁杩囪繎
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_璺濈", 1);
    return nullptr;
  }

  // 妫€鏌ョ洰鏍囩偣鏄惁鏋勯€犳棤鏁堢殑鏃嬭浆鐩爣
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_鏃嬭浆", 1);
    return nullptr;
  }

  // frc::Rotation2d current_angle = currentPose.Rotation();
  frc::Rotation2d target_angle = targetPose.Rotation();

  // double vx = GetState().Speeds.vx();
  // double vy = GetState().Speeds.vy();
  // double v = sqrt(vx * vx + vy * vy);
  // frc::Rotation2d start_heading = frc::Rotation2d{units::radian_t{atan2(vy,
  // vx)}}; frc::Rotation2d end_heading = targetPose.Translation().Angle();

  // 鍙栧綋鍓嶄綅濮垮拰鐩爣浣嶅Э鐨勪腑鐐逛綔涓鸿矾寰勭殑绗竴涓獁aypoint,娉ㄦ剰姝ゆ椂waypoints绗笁涓€紃otation骞堕潪涓哄簳鐩樻棆杞搴﹁€屾槸鏈哄櫒浜烘鏃剁殑琛岃繘鏈濆悜
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 涓ょ偣涔嬮棿鐩稿浜巟杞寸殑瑙掑害
  // frc::Rotation2d waypoint_heading = temp_heading.Degrees() > 0_deg ?
  // (-180_deg + temp_heading.Degrees()) : (180_deg + temp_heading.Degrees());
  // // 璁＄畻鐩寸嚎琛岄┒鏃剁殑琛岃繘鏈濆悜

  // frc::SmartDashboard::PutNumber("temp_heading",
  // temp_heading.Degrees().value());
  // frc::SmartDashboard::PutNumber("waypoint_heading",
  // waypoint_heading.Degrees().value());

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 璧峰锟?

  frc::Pose2d waypoint1{
      (currentPose.Translation() + targetPose.Translation()) / 2.0,
      waypoint_heading};  // 涓ょ偣涔嬮棿鐨勪腑锟?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          waypoint_heading};  // 鐩爣锟?

  // 鍒涘缓鍔ㄦ€佽矾寰勭殑waypoints
  std::vector<frc::Pose2d> poses = {startpoint, waypoint1, targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 璁剧疆璺緞绾︽潫锛堝彲浠ユ牴鎹渶瑕佽皟鏁存渶澶ч€熷害鍜屽姞閫熷害锟?/2
  PathConstraints constraints(_maxspeed * 1_mps, _maxacc * 1_mps_sq,
                              540_deg_per_s, 980_deg_per_s_sq);
  // frc::SmartDashboard::PutNumber("V", v);
  // frc::SmartDashboard::PutNumber("start_heading",
  // start_heading.Degrees().value()); PathConstraints constraints(0.7_mps,
  // 0.7_mps_sq, 540_deg_per_s, 980_deg_per_s_sq); 鍒涘缓璺緞
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints,
      // IdealStartingState(0_mps, current_angle), //
      // 绌虹殑璧峰鐘舵€侊紝瀵逛簬鍔ㄦ€佺敓鎴愮殑璺緞锛屾垜浠笉闇€瑕佺悊鎯崇殑璧峰鐘讹拷?
      std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 鐩爣鐘舵€侊紝杩欓噷璁剧疆涓虹粰瀹氱洰鏍囬€熷害鍜岃锟?
  );
  // frc::SmartDashboard::PutNumberArray("path.poses",
  // path.get()->bezierFromPoses[1]); 闃叉璺緞鍦ㄦ纭殑鍧愭爣涓嬭缈昏浆
  path->preventFlipping = true;

  // path123 = *path;

  // 璁板綍鏁版嵁
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
 * @brief 浠庡涓洰鏍囩偣鐢熸垚璺緞
 *
 * @param targetPoses Pose2d鏁扮粍,鍏朵腑:
 *   - 鍓峃-1涓偣鐨凴otation琛ㄧず**琛岃繘鏂瑰悜(heading)**
 *   - 鏈€锟?涓偣鐨凴otation琛ㄧず**鏈€缁堝簳鐩樻湞锟?target rotation)**
 *
 * @return 鐢熸垚鐨勮矾锟?澶辫触杩斿洖nullptr
 *
 * @example
 * std::vector<frc::Pose2d> waypoints = {
 *     {2_m, 3_m, 45_deg},   // 锟?5掳鏂瑰悜琛岃繘
 *     {4_m, 5_m, 90_deg},   // 锟?0掳鏂瑰悜琛岃繘
 *     {6_m, 7_m, 180_deg}   // 鍒拌揪鍚庤溅澶存湞180掳
 * };
 */
std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::GeneratePath(
    std::vector<frc::Pose2d> const& targetPoses) {
  if (targetPoses.empty()) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_鏃犵洰鏍囩偣", 1);
    return nullptr;
  }

  frc::Pose2d currentPose = GetState().Pose;

  // 楠岃瘉绗竴涓偣璺濈
  double deltaX = targetPoses[0].Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPoses[0].Translation().Y().value() -
                  currentPose.Translation().Y().value();
  if (std::sqrt(deltaX * deltaX + deltaY * deltaY) < 0.05) {
    return nullptr;
  }

  // 锟?鏋勫缓璺緞鐐瑰垪锟?
  std::vector<frc::Pose2d> poses;

  // 璧风偣: 鏈濆悜绗竴涓洰锟?
  frc::Rotation2d initial_heading =
      (targetPoses[0].Translation() - currentPose.Translation()).Angle();
  poses.push_back({currentPose.Translation(), initial_heading});

  // 锟?鍏抽敭淇敼: 鐩存帴浣跨敤鐢ㄦ埛浼犲叆鐨凴otation浣滀负heading
  for (size_t i = 0; i < targetPoses.size(); ++i) {
    poses.push_back(targetPoses[i]);
  }

  // 锟?鍒涘缓璺緞
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 鏈€鍚庝竴涓偣鐨凴otation鏃㈡槸heading涔熸槸target_rotation
  frc::Rotation2d final_target_rotation = targetPoses.back().Rotation();

  // PathConstraints constraints(2.5_mps, 4_mps_sq, 540_deg_per_s,
  // 690_deg_per_s_sq);
  PathConstraints constraints(1.8_mps, 1.8_mps_sq, 640_deg_per_s,
                              980_deg_per_s_sq);

  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints, std::nullopt,
      GoalEndState(0_mps, final_target_rotation)  // 浣跨敤鏈€鍚庝竴涓偣鐨凴otation
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

// 鐐圭増锟?- 鏂板
frc2::CommandPtr CommandSwerveDrivetrain::followPathCommand(
    std::vector<frc::Pose2d> const& targetPoses) {
  static double failtime = 0;
  static double successtime = 0;

  // 璋冪敤澶氱偣鐗堟湰锟?GeneratePath
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
  // 鑾峰彇褰撳墠鏈哄櫒浜虹殑浣嶅Э
  frc::Pose2d currentPose = GetState().Pose;
  frc::SmartDashboard::PutNumber("AAAMAEK",
                                 currentPose.Rotation().Degrees().value());

  // 浣跨敤璺濈妫€鏌ョ洰鏍囧拰褰撳墠鏄惁杩囪繎
  double deltaX = targetPose.Translation().X().value() -
                  currentPose.Translation().X().value();
  double deltaY = targetPose.Translation().Y().value() -
                  currentPose.Translation().Y().value();
  double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < 0.05) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_璺濈", 1);
    return nullptr;
  }

  // 妫€鏌ョ洰鏍囩偣鏄惁鏋勯€犳棤鏁堢殑鏃嬭浆鐩爣
  auto targetRotation = targetPose.Rotation();
  double targetCos = targetRotation.Cos();
  double targetSin = targetRotation.Sin();

  if (std::abs(targetCos) < 0.001 && std::abs(targetSin) < 0.001) {
    frc::SmartDashboard::PutNumber("杞ㄨ抗鍑芥暟鍒涘缓澶辫触_鏃嬭浆", 1);
    return nullptr;
  }

  // 鑾峰彇褰撳墠鍜岀洰鏍囩偣鐨勫簳鐩樻棆杞锟?
  frc::Rotation2d target_angle = targetPose.Rotation();

  // 鍙栧綋鍓嶄綅濮垮拰鐩爣浣嶅Э鐨勪腑鐐逛綔涓鸿矾寰勭殑绗竴涓獁aypoint,娉ㄦ剰姝ゆ椂waypoints绗笁涓€紃otation骞堕潪涓哄簳鐩樻棆杞搴﹁€屾槸鏈哄櫒浜烘鏃剁殑琛岃繘鏈濆悜
  frc::Rotation2d waypoint_heading =
      (targetPose.Translation() - currentPose.Translation())
          .Angle();  // 涓ょ偣涔嬮棿鐩稿浜巟杞寸殑瑙掑害
  frc::Rotation2d targetpoint_heading =
      target_angle;  // 鏈€鍚庣偣鐨勮杩涙湞鍚戝簲涓虹洰鏍囪锟?

  frc::Pose2d startpoint{
      // frc::Translation2d{0.5_m, 0.5_m},
      currentPose.Translation(), waypoint_heading};  // 璧峰锟?

  frc::Pose2d targetpoint{targetPose.Translation(),
                          targetpoint_heading};  // 鐩爣锟?
  // targetpoint_heading 鏄寚锟?Reef
  // 鐨勬柟鍚戯紝閫€锟?.6m鍒涘缓涓€涓獁aypoint,閰嶅悎瑙嗚妯″紡2
  frc::Pose2d Reefpoint{
      targetPose.Translation() -
          frc::Translation2d{targetpoint_heading.Cos() * 0.75_m,
                             targetpoint_heading.Sin() * 0.6_m},
      targetpoint_heading};

  // 鍒涘缓鍔ㄦ€佽矾寰勭殑waypoints
  std::vector<frc::Pose2d> poses = {startpoint, Reefpoint,
                                    // waypoint1,
                                    targetpoint};
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 璁剧疆璺緞绾︽潫锛堝彲浠ユ牴鎹渶瑕佽皟鏁存渶澶ч€熷害鍜屽姞閫熷害锟?/2
  PathConstraints constraints(1.8_mps, 1.8_mps_sq, 640_deg_per_s,
                              980_deg_per_s_sq);

  // 鍒涘缓璺緞
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints, std::nullopt,
      GoalEndState(0_mps,
                   target_angle)  // 鐩爣鐘舵€侊紝杩欓噷璁剧疆涓虹粰瀹氱洰鏍囬€熷害鍜岃锟?
  );

  // 闃叉璺緞鍦ㄦ纭殑鍧愭爣涓嬭缈昏浆
  path->preventFlipping = true;

  // 鍒涘缓骞舵坊锟?EventMarker
  // ReefPoint 鏄 2 涓偣 (绱㈠紩锟?1)锛屾垜浠笇鏈涘湪鍒拌揪 ReefPoint 鏃惰Е鍙戝懡锟?
  path->getEventMarkers().push_back(pathplanner::EventMarker(
      "event1_trigger",                              // 瑙﹀彂鍣ㄥ悕锟?
      1.0,                                           // 浣嶇疆 (锟?2 涓偣)
      frc2::cmd::RunOnce([this] { eventflag = 1; })  // 浼犲叆command
      ));

  // 鍒涘缓骞舵坊锟?
  // RotationTarget,鍦╮eefpoint鍗冲瀭鐩寸強鐟氱锛屼娇绗簩娈典粎璋冭妭鍚戝墠鐨勯噺锛屽苟涓旀惌閰嶆贩鍚堣瑙夋ā寮忕籂姝ｆ満鍣ㄤ汉鏃嬭浆鏂瑰悜
  path->getRotationTargets().push_back(pathplanner::RotationTarget(
      1.0,          // 浣嶇疆 (锟?2 涓偣)
      target_angle  // 鐩爣鏃嬭浆瑙掑害
      ));

  return path;
}

// DriveAiming瀹炵幇

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
        m_driveAimingRequest.WithHeadingPID(8.0, 0, 0.1)
            .WithRotationalDeadband(units::radians_per_second_t{0.05})
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

double CommandSwerveDrivetrain::GetDistanceToHub() {
  const auto robotPose = GetState().Pose;
  const auto hubPos = GetHubPosition();
  const double dx = (hubPos.X() - robotPose.X()).value();
  const double dy = (hubPos.Y() - robotPose.Y()).value();
  return std::hypot(dx, dy);
}

double CommandSwerveDrivetrain::NormalizeAngle(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

//在pathplanner library里面，pose2d是机器人移动切线方向，而机器人朝向需要另外计算
std::shared_ptr<PathPlannerPath> CommandSwerveDrivetrain::GenerateShootOnMovePath(std::vector<frc::Pose2d> const& targetPoses) {
  //后面的逻辑不允许少于两个点
  if(targetPoses.size()<=1){
    return nullptr;
  }

  std::vector<frc::Pose2d> poses;
  frc::Pose2d currentPose = GetState().Pose;

  units::meters_per_second_t vx = GetState().Speeds.vx; 
  units::meters_per_second_t vy = GetState().Speeds.vy;
  units::meters_per_second_t startSpeed = units::math::hypot(vx, vy);
  pathplanner::IdealStartingState startState(startSpeed, currentPose.Rotation());
  
  frc::Rotation2d initial_heading;
  if (startSpeed > 0.1_mps) {
    // 运动中切线方向完全由 vx 和 vy 的比例决定
    initial_heading= frc::Rotation2d(units::math::atan2(vy, vx));
  } else {
    // 静止时直接把切线方向指向我们的第一个目标点
    initial_heading = (targetPoses[0].Translation() - currentPose.Translation()).Angle();
  }

  //第一个pose应该是机器人当前位置和移动方向
  poses.push_back({currentPose.Translation(), initial_heading});

  //希望在抵达waypoint的时候方向朝向下一个waypoint
  for (size_t i = 0; i < targetPoses.size()-1; i++) {
    poses.push_back({targetPoses[i].Translation(), (targetPoses[i+1].Translation() - targetPoses[i].Translation()).Angle()});
  }
  //最后一个点
  poses.push_back({targetPoses[targetPoses.size()-1].Translation(), (targetPoses[targetPoses.size()-1].Translation()-targetPoses[targetPoses.size()-2].Translation()).Angle()});

  // 生成waypoints
  std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

  // 先把最后一个点的机器人面向方向加入goalendstate，在生成路径后加入机器人面向的方向
  frc::Rotation2d final_target_rotation = targetPoses.back().Rotation();

  // PathConstraints constraints(2.5_mps, 4_mps_sq, 540_deg_per_s,
  // 690_deg_per_s_sq);
  PathConstraints constraints(0.7_mps, 1.8_mps_sq, 640_deg_per_s,
                              980_deg_per_s_sq);

  //生成路径
  auto path = std::make_shared<PathPlannerPath>(
      waypoints, constraints, startState,
      GoalEndState(0_mps, final_target_rotation)  
  );

  //加入waypoint的机器人面朝方向
  for(int i=0; i<targetPoses.size()-1; i++){
    double waypointIndex = static_cast<double>(i + 1);
    pathplanner::RotationTarget rotTarget(waypointIndex, targetPoses[i].Rotation());
    path->getRotationTargets().push_back(rotTarget);
  }
  path->preventFlipping = true;

  return path;
}

frc2::CommandPtr CommandSwerveDrivetrain::followShootOnMovePathCommand(int direction){
  frc::Pose2d currentPose=GetState().Pose;
  //设置每个路径点的距离，理论上距离越近精度越高也会更消耗cpu。 单位m
  double disBetweenPoints=0.7;
  std::vector<frc::Translation2d> pointsDiff;
  std::vector<frc::Rotation2d> targetRot;
  std::vector<frc::Pose2d> poses;
  double targetDirection=-NormalizeAngle(direction);
  // switch(direction){
  //   case 0: targetDirection=0; break;
  //   case 1: targetDirection=90; break;//1走左边
  //   case 2: targetDirection=0; break;//2 is forward
  //   case 3: targetDirection=180; break;//3 is backward
  //   case 4: targetDirection=-45; break;// 4 is topright
  //   case 5: targetDirection=135; break;//5 is backleft
  //   case 6: targetDirection=45; break;//6 is topleft
  //   case 7: targetDirection=-135; break;//7 is backright
  //   default: return frc2::cmd::None();//如果数字不对return空命令
  // }
  frc::Rotation2d rot{units::degree_t(targetDirection)};
  int count=1;
  const int maxcount=(int)(8/disBetweenPoints);//最多走8米
  double rotContainer;
  while(count<=maxcount){
    frc::Translation2d transl(units::meter_t{disBetweenPoints*count}, rot);
    pointsDiff.push_back(currentPose.Translation()+transl);
    rotContainer=atan2(GetHubPosition().Y().value()-pointsDiff[count-1].Y().value(),GetHubPosition().X().value()-pointsDiff[count-1].X().value() );
    targetRot.push_back({units::degree_t{NormalizeAngle(rotContainer/PI*180)}});
    poses.push_back({pointsDiff[count-1], targetRot[count-1]});
    count++;
  }
  auto path=GenerateShootOnMovePath(poses);
  if(path!=nullptr){
    return AutoBuilder::followPath(path);
  }
  else{
    return frc2::cmd::None();
  }
}


void CommandSwerveDrivetrain::changeDriveCurrentLimit(double newLim) {
  // 启动一个后台线程 
  std::thread([this, newLim]() {
      ctre::phoenix6::configs::CurrentLimitsConfigs current_limits{};
      current_limits.SupplyCurrentLimit = newLim*1_A; 
      current_limits.SupplyCurrentLimitEnable = true;
      current_limits.StatorCurrentLimit = 90.0_A;
      current_limits.StatorCurrentLimitEnable = true;

      ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
      for (int i = 0; i < 4; ++i) {
        for(int j = 0; j < 5; j++){
          status = GetModule(i).GetDriveMotor().GetConfigurator().Apply(current_limits);
          if(status.IsOK()){
            break; 
          }
        }
      }
  }).detach(); 
}