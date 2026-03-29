#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <cmath>
#include "commands/PassBallCommand.h"
PassBallCommand::PassBallCommand(CommandSwerveDrivetrain* drive, 
                                 ShooterSubsystem* shooter,
                                 FeederSubsystem* feeder,
                                
                                 std::function<double()> vx, 
                                 std::function<double()> vy, double AOS)
    : m_drive(drive), 
      m_feeder(feeder),
      m_shooter(shooter),
      m_vXSupplier(vx), 
      m_vYSupplier(vy),
      angleOfShooter(AOS) {
  
  // 🌟 核心：向调度器声明这个命令占用了这 4 个子系统
  // 这样在传球时，任何其他试图使用它们的命令都会被自动阻挡或打断
  AddRequirements({drive,feeder}); 
}
void PassBallCommand::Initialize() {
  // 1. 激活 Shooter 的传球模式，加载固定的长传速度和仰角
  m_shooter->startPassing(); 

  // 2. 物理安全锁：既然独占了地吸和输送带，启动瞬间强行让它们停止
  // （请确保你的 Subsystem 里有类似 Stop() 的方法，名字根据你的实际代码修改）
  // m_groundIntake->Stop(); 
  // m_feeder->Stop(); 
    //m_timer.Start();
  // 3. 初始化底盘的 PID 和死区配置，保证瞄准丝滑
  driveClosed.WithHeadingPID(9, 0, 0.1)
             .WithDeadband(MaxSpeed * 0.05)
             .WithRotationalDeadband(units::radians_per_second_t{0.1})
             .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14})
             .WithDriveRequestType(swerve::DriveRequestType::Velocity)
             .WithSteerRequestType(swerve::SteerRequestType::Position);


}

// ==========================================
// 2. Execute：按住按钮期间，每 20ms 循环执行
// ==========================================
void PassBallCommand::Execute() {
  frc::ChassisSpeeds robotSpeeds = m_drive->GetState().Speeds;
  frc::Pose2d currentPose = m_drive->GetState().Pose;
  frc::Rotation2d robotHeading = currentPose.Rotation();

  // 把车身速度变成场地绝对速度，并消除极小噪声
  double vx = robotSpeeds.vx.value() * robotHeading.Cos() - robotSpeeds.vy.value() * robotHeading.Sin();
  double vy = robotSpeeds.vx.value() * robotHeading.Sin() + robotSpeeds.vy.value() * robotHeading.Cos();
  if (std::sqrt(vx * vx + vy * vy) < 0.05) {
      vx = 0.0; vy = 0.0;
  }

  // 获取手柄输入
  double nowX = frc::ApplyDeadband(m_vXSupplier(), 0.05) * MaxSpeed.value() * 0.3;
  double nowY = frc::ApplyDeadband(m_vYSupplier(), 0.05) * MaxSpeed.value() * 0.3;

  bool isRed = frc::DriverStation::GetAlliance().has_value() && 
               frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed;

  double realX = isRed ? -nowX : nowX;
  double realY = isRed ? -nowY : nowY;

  // 预测 0.4 秒后的未来位置
  double latencySeconds = 0.4;
  double predictedX = currentPose.X().value() + (realX * latencySeconds);
  double predictedY = currentPose.Y().value() + (realY * latencySeconds);

  // 🌟 核心战术逻辑：根据机器人的 Y 坐标判断我们在哪一边，自动选择传球落点
  // 假设场地 Y 轴中线是 4.1 米
  frc::Translation2d targetPoint;
  if (currentPose.Y().value() > 4) {
      targetPoint = kBlueLeftTarget;  // 左半场，面向左侧传球点
  } else {
      targetPoint = kBlueRightTarget; // 右半场，面向右侧传球点
  }

  // 如果是红方，镜像落点的 X 坐标 (假设场地全长约 16.54 米)
  if (isRed) {
      targetPoint = frc::Translation2d{
          units::meter_t{16.54 - targetPoint.X().value()},
          targetPoint.Y()
      };
  }
  m_shooter->setPassTarget(targetPoint);
  // 计算对准落点的目标角度
  double dx = targetPoint.X().value() - predictedX;
  double dy = targetPoint.Y().value() - predictedY;
  double targetAngleRad = atan2(dy, dx);

  // 计算底盘速度带来的抛物线横向偏移
  double Normvx = realX * cos(-targetAngleRad) + realY * cos(PI/2 - targetAngleRad);
  double Normvy = realX * sin(-targetAngleRad) + realY * sin(PI/2 - targetAngleRad);
  
  double shootCoeff = 0.17; 
  
  // 这时候 m_shooter->vel 已经是我们在 Init 里触发的传球专用速度了
  double Shootvx = m_shooter->vel * cos(m_shooter->Tangle / 180.0 * PI) * shootCoeff;
  double Shootvz = m_shooter->vel * sin(m_shooter->Tangle / 180.0 * PI) * shootCoeff;

  Shootvx = Shootvx - Normvx;
  Normvy = -Normvy;
  
  // 算出需要给飞轮补偿的速度和角度
  double ShooterVelOff = sqrt(Shootvx*Shootvx + Normvy*Normvy + Shootvz*Shootvz) / shootCoeff - m_shooter->vel;
  double chassisAngleOffset = 0;
  if (m_shooter->vel >= 10) {
      chassisAngleOffset = atan2(Normvy, Shootvx);
  }
  
  targetAngleRad += chassisAngleOffset;
  if (isRed) {
      targetAngleRad -= PI;
  }
  targetAngleRad-=angleOfShooter/180*PI;

  double shootAngleOffset = atan2(Shootvz, Shootvx) / PI * 180 - m_shooter->Tangle;
  m_shooter->SetAngleOffset(shootAngleOffset);
  m_shooter->SetSpeedOffset(ShooterVelOff);
  //没被hub挡住时启动feeder
  bool rightPos=currentPose.Y().value()<=3.5||currentPose.Y().value()>=4.5;
  //同时飞轮速度达标
  bool rightSpeed=std::abs(m_shooter->realShootVelocity-m_shooter->GetShootVelocity())<=10;
  frc::SmartDashboard::PutNumber("shootOnMove/passShooterVreal",m_shooter->GetShootVelocity());
  frc::SmartDashboard::PutNumber("shootOnMove/passShooterVexpected",m_shooter->realShootVelocity);
  //同时底盘旋转ok
  bool rightRot=m_drive->SOMangleDiff<=13;

  frc::SmartDashboard::PutBoolean("shootOnMove/RightPos", rightPos);
frc::SmartDashboard::PutBoolean("shootOnMove/RightSpeed", rightSpeed);
frc::SmartDashboard::PutBoolean("shootOnMove/RightRot", rightRot);
  if(rightPos&&rightSpeed&&rightRot){
    m_feeder->SetBackwardFeederVelocity(1);
    m_feeder->SetUpwardFeederVelocity(FeederConstants::kUpwardVelocityTarget
    );
  }
  else{
     m_feeder->setduty(0,0);
  }
  // 下发控制指令到底盘
  frc::Rotation2d rott{units::radian_t(targetAngleRad)};
  m_drive->SetControl(
      driveClosed.WithVelocityX(units::meters_per_second_t{nowX})
                 .WithVelocityY(units::meters_per_second_t{nowY})
                 .WithTargetDirection(rott)
  );
  
  frc::Rotation2d tlow{units::radian_t(currentPose.Rotation().Radians().value())};
  double angledi=std::abs((rott-tlow).Degrees().value());
  if(isRed){
    angledi=180-angledi;
  }
  frc::SmartDashboard::PutNumber("shootOnMove/AngleDiff",angledi );
  m_drive->SOMangleDiff = angledi;
}

// ==========================================
// 3. End：松开按钮瞬间触发
// ==========================================
void PassBallCommand::End(bool interrupted) {
  // 1. 退出传球模式，恢复自动打 Speaker 的状态
  m_shooter->stopPassing(); 
   m_feeder->SetBackwardFeederVelocity(0);
    m_feeder->SetUpwardFeederVelocity(0);
  // 2. 清空所有的射击补偿偏移
  m_shooter->SetAngleOffset(0);
  m_shooter->SetSpeedOffset(0);
  
  // 3. 让底盘安全滑行停下
  auto currentSpeeds = m_drive->GetState().Speeds;
  m_drive->SetControl(
      m_drive->m_safeCoastRequest
          .WithVelocityX(currentSpeeds.vx)
          .WithVelocityY(currentSpeeds.vy)
          .WithRotationalRate(currentSpeeds.omega)
  );
}

// ==========================================
// 4. IsFinished：永远返回 false，配合 WhileTrue
// ==========================================
bool PassBallCommand::IsFinished() {
  return false; 
}