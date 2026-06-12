#include "commands/AutoRealTimeAimDrive.h"
#include <numbers>
#include <cmath>

AutoRealTimeAimDrive::AutoRealTimeAimDrive(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, FeederSubsystem* fed, double AOSDeg)
    : m_drive(drive), m_shooter(sh), feeder(fed), angleOfShooter(AOSDeg) {

}

void AutoRealTimeAimDrive::Initialize() {
  m_drive->autoShooting = true;
}

// ==========================================
// Execute：只做数学计算，不控制电机
// ==========================================
void AutoRealTimeAimDrive::Execute() {
  frc::ChassisSpeeds robotSpeeds = m_drive->GetState().Speeds;
  frc::Rotation2d robotHeading = m_drive->GetState().Pose.Rotation();

  // 1. 获取场地绝对速度
  double vx = robotSpeeds.vx.value() * robotHeading.Cos() - robotSpeeds.vy.value() * robotHeading.Sin();
  double vy = robotSpeeds.vx.value() * robotHeading.Sin() + robotSpeeds.vy.value() * robotHeading.Cos();
  
  // 消除噪声
  if (std::sqrt(vx * vx + vy * vy) < 0.05) {
      vx = 0.0;
      vy = 0.0;
  }

  // 2. 获取当前坐标并计算真实的 Hub 角度
  frc::Pose2d currentPose = m_drive->GetState().Pose;
  double curX = currentPose.X().value();
  double curY = currentPose.Y().value();
  double hubX = m_drive->GetHubPosition().X().value();
  double hubY = m_drive->GetHubPosition().Y().value();

  double dx_current = hubX - curX;
  double dy_current = hubY - curY;
  double currentAngleToHubRad = std::atan2(dy_current, dx_current);

  // 3. 计算预测位置 (基于当前真实速度)
  // v_radial: 径向速度 (正代表靠近 Hub，负代表远离)  
  double v_radial = vx * cos(currentAngleToHubRad) + vy * sin(currentAngleToHubRad);
  // v_tangential: 切向/横向速度 (正代表逆时针绕 Hub 走)
  double v_tangential = -vx * sin(currentAngleToHubRad) + vy * cos(currentAngleToHubRad);

  double latencyRadialSeconds = 0.3;     // 竖向（靠近/远离）的预测时间
  double latencyTangentialSeconds = -0.1; // 横向（绕圈）的预测时间

  double d_radial = v_radial * latencyRadialSeconds;
  double d_tangential = v_tangential * latencyTangentialSeconds;

  double deltaX = d_radial * cos(currentAngleToHubRad) - d_tangential * sin(currentAngleToHubRad);
  double deltaY = d_radial * sin(currentAngleToHubRad) + d_tangential * cos(currentAngleToHubRad);

  double predictedX = curX + deltaX;
  double predictedY = curY + deltaY;

  // 4. 基于预测位置计算新角度
  double dx_pred = hubX - predictedX;
  double dy_pred = hubY - predictedY;
  double targetAngleRad = std::atan2(dy_pred, dx_pred);

  // 5. 射击补偿 (SOTM 底层数学)
  // 把底盘即时转换到以机器人与hub的连线为0度的速度向量
  double Normvx = vx * cos(-targetAngleRad) + vy * cos(PI/2 - targetAngleRad);
  double Normvy = vx * sin(-targetAngleRad) + vy * sin(PI/2 - targetAngleRad);
  
  double shootCoeff = 0.25;
  
  // 把射球的速度转换成向量
  double Shootvx = m_shooter->vel * cos(m_shooter->Tangle / 180.0 * PI) * shootCoeff;
  double Shootvz = m_shooter->vel * sin(m_shooter->Tangle / 180.0 * PI) * shootCoeff;
  
  // 合并向量
  Shootvx = Shootvx - Normvx;
  Normvy = -Normvy;
  
  double ShooterVelOff = sqrt(Shootvx*Shootvx + Normvy*Normvy + Shootvz*Shootvz) / shootCoeff - m_shooter->vel;
  double chassisAngleOffset = 0;
  if(m_shooter->vel >= 10) {
      chassisAngleOffset = atan2(Normvy, Shootvx);
  }
  
  targetAngleRad += chassisAngleOffset;
  targetAngleRad -= angleOfShooter / 180.0 * PI;

  // 联盟翻转判定
  if(frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed){
    targetAngleRad -= PI;
  }

  // 限制角度在 -PI 到 PI 之间
  while(targetAngleRad <= -PI) targetAngleRad += 2 * PI;
  while(targetAngleRad > PI) targetAngleRad -= 2 * PI;

  double shootAngleOffset = atan2(Shootvz, Shootvx) / PI * 180 - m_shooter->Tangle;
  m_shooter->SetAngleOffset(shootAngleOffset);
  m_shooter->SetSpeedOffset(ShooterVelOff);

  // 6. 计算误差角度，用于判断是否可以射击
  double currentAngleRad = robotHeading.Radians().value();
  frc::Rotation2d ttall{units::radian_t(targetAngleRad)};
  frc::Rotation2d tlow{units::radian_t(currentAngleRad)};
  double angledi = std::abs((ttall - tlow).Degrees().value());

  if(frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed){
    angledi = 180 - angledi;
  }

  m_drive->SOMangleDiff = angledi;

  m_drive->autoRot = targetAngleRad/PI*180; 
  
  // Debug Outputs
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseX", curX);
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseY", curY);
  frc::SmartDashboard::PutNumber("shootOnMove/PredictedX", predictedX);
  frc::SmartDashboard::PutNumber("shootOnMove/PredictedY", predictedY);
  frc::SmartDashboard::PutNumber("shootOnMove/AngleDiff", angledi);
  frc::SmartDashboard::PutNumber("shootOnMove/AutoTargetAngleRad", targetAngleRad);
}

// ==========================================
// End：释放状态
// ==========================================
void AutoRealTimeAimDrive::End(bool interrupted) {
  m_shooter->SetAngleOffset(0);
  m_shooter->SetSpeedOffset(0);
  m_drive->autoShooting = false;
  m_drive->SOMangleDiff = 100; // 设为一个极大的值，防止误开火
}

// ==========================================
// IsFinished：配合 RaceWith 使用，永远不自动结束
// ==========================================
bool AutoRealTimeAimDrive::IsFinished() {
  return false; 
}