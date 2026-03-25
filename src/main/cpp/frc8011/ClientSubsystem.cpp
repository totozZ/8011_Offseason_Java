#include "frc8011/ClientSubsystem.h"

#include <pathplanner/lib/util/PathPlannerLogging.h>

using namespace subsystems;

ClientSubsystem::ClientSubsystem(
    subsystems::CommandSwerveDrivetrain* driveSubsystem)
    : m_drivesubsystem(driveSubsystem) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_table = inst.GetTable("client");

  // 设置在 Dashboard 上让操作手选择谁最先射击
  m_firstAttackerChooser.SetDefaultOption("OUR Alliance First", true);
  m_firstAttackerChooser.AddOption("OPPONENT Alliance First", false);
  frc::SmartDashboard::PutData("First Shoot Setting", &m_firstAttackerChooser);

  frc::SmartDashboard::PutData("Field", &m_field);

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](auto poses) { m_field.GetObject("path")->SetPoses(poses); });

  //   targetPoseSub =
  //   m_table->GetDoubleArrayTopic("targetPose").Subscribe(std::vector<double>{0.0,
  //   0.0, 0.0});
  // levelSub = m_table->GetIntegerTopic("targetLevel").Subscribe(2);
  // is_completedPub = m_table->GetBooleanTopic("isCompleted").Publish();

  // Start WebServer for Elastic Dashboard "Download from robot" layout config
  wpi::WebServer::GetInstance().Start(5800,
                                      frc::filesystem::GetDeployDirectory());
}

void ClientSubsystem::Periodic() {
  try {
    // 实时更新机器人本体在场地上的位置！
    if (m_drivesubsystem != nullptr) {
      auto currentPose = m_drivesubsystem->GetState().Pose;
      m_field.SetRobotPose(currentPose);
    }

    // getLevel();
    // getTargetPose();

    // std::vector<double> targetData = {
    //         targetPose.Translation().X().value(),
    //                                   targetPose.Translation().Y().value(),
    //                                   targetPose.Rotation().Degrees().value(),
    //                                   static_cast<double>(GetLevel())};
    // frc::SmartDashboard::PutNumberArray("client_target_data", targetData);

    // frc::SmartDashboard::PutNumber("client_level", GetLevel());

    // Publish central match telemetry parameters
    frc::SmartDashboard::PutNumber("MatchTime",
                                   frc::DriverStation::GetMatchTime().value());
    frc::SmartDashboard::PutNumber("BatteryVoltage",
                                   frc::DriverStation::GetBatteryVoltage());
    UpdateMatchPhaseCountdown();

  } catch (const std::exception& e) {
    frc::SmartDashboard::PutString("ClientSubsystem Periodic Failed:",
                                   e.what());
  }
}

void ClientSubsystem::UpdateMatchPhaseCountdown() {
  // 从 Dashboard 获取用户的选择
  is_our_turn_first = m_firstAttackerChooser.GetSelected();

  double match_time = frc::DriverStation::GetMatchTime().value();
  bool is_auto = frc::DriverStation::IsAutonomousEnabled();
  bool is_teleop = frc::DriverStation::IsTeleopEnabled();

  if (match_time < 0 || (!is_auto && !is_teleop)) {
    frc::SmartDashboard::PutString("Match_Phase", "Waiting/Disabled");
    frc::SmartDashboard::PutNumber("Phase_Countdown", 0.0);
    frc::SmartDashboard::PutBoolean("Can_Shoot_Now", false);
    return;
  }

  ClientSubsystem::MatchPhase current_phase =
      ClientSubsystem::MatchPhase::kUnknown;
  double countdown = 0.0;
  std::string phase_string = "Unknown";
  bool can_i_shoot = false;

  // 1. 自动阶段 (AUTO): 倒计时 20 -> 0
  if (is_auto) {
    current_phase = ClientSubsystem::MatchPhase::kAuto;
    countdown = match_time;
    phase_string = "AUTO (Both Active)";
    can_i_shoot = true;
  }
  // 2. 手动阶段 (TELEOP): 倒计时 140 (2:20) -> 0
  else if (is_teleop) {
    if (match_time > 130.0) {  // 2:20 - 2:10
      current_phase = ClientSubsystem::MatchPhase::kTransition;
      countdown = match_time - 130.0;
      phase_string = "Transition (Both)";
      can_i_shoot = true;
    } else if (match_time >= 105.0 && match_time <= 130.0) {
      current_phase = ClientSubsystem::MatchPhase::kSwitch1;
      countdown = match_time - 105.0;
      can_i_shoot = is_our_turn_first;
      phase_string =
          can_i_shoot ? "Switch 1 (OUR Turn)" : "Switch 1 (OPPONENT)";
    } else if (match_time >= 80.0 && match_time < 105.0) {
      current_phase = ClientSubsystem::MatchPhase::kSwitch2;
      countdown = match_time - 80.0;
      can_i_shoot = !is_our_turn_first;  // 交换逻辑
      phase_string =
          can_i_shoot ? "Switch 2 (OUR Turn)" : "Switch 2 (OPPONENT)";
    } else if (match_time >= 55.0 && match_time < 80.0) {
      current_phase = ClientSubsystem::MatchPhase::kSwitch3;
      countdown = match_time - 55.0;
      can_i_shoot = is_our_turn_first;
      phase_string =
          can_i_shoot ? "Switch 3 (OUR Turn)" : "Switch 3 (OPPONENT)";
    } else if (match_time >= 30.0 && match_time < 55.0) {
      current_phase = ClientSubsystem::MatchPhase::kSwitch4;
      countdown = match_time - 30.0;
      can_i_shoot = !is_our_turn_first;
      phase_string =
          can_i_shoot ? "Switch 4 (OUR Turn)" : "Switch 4 (OPPONENT)";
    } else if (match_time < 30.0) {
      current_phase = ClientSubsystem::MatchPhase::kEndgame;
      countdown = match_time;
      phase_string = "ENDGAME (Both)";
      can_i_shoot = true;
    }
  }

  frc::SmartDashboard::PutString("Match_Phase", phase_string);
  frc::SmartDashboard::PutNumber("Phase_Countdown", countdown);
  frc::SmartDashboard::PutBoolean("Can_Shoot_Now", can_i_shoot);
}