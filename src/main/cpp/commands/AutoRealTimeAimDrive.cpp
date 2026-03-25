#include "commands/AutoRealTimeAimDrive.h"
#include <numbers> 

AutoRealTimeAimDrive::AutoRealTimeAimDrive(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, FeederSubsystem* fed, double x, double y, bool one, bool two,double AOSDeg)
    : m_drive(drive), m_shooter(sh), feeder(fed),targetX(x), targetY(y), invertA(one), invertD(two),angleOfShooter(AOSDeg){
  
  AddRequirements({drive}); // 声明占用底盘
  
  // 告诉 PID -180度和180度是相连的
  //m_aimPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void AutoRealTimeAimDrive::Initialize() {
  //m_aimPID.Reset(); 
  if(invertA){
    targetX=16.54-targetX;
  }
  if(invertD){
    targetY=8.07-targetY;
  }
  m_xPID.Reset();
  m_yPID.Reset();
  driveClosed.WithHeadingPID(7, 0, 0.1)
                    .WithDeadband(MaxSpeed * 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})
                    
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14*1.5})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
  m_targetWaypoint={units::meter_t{targetX},units::meter_t{targetY}, frc::Rotation2d{units::degree_t{0}}};
}

// ==========================================
// Execute：按住按钮时，每 20ms 疯狂执行的核心逻辑
// ==========================================
void AutoRealTimeAimDrive::Execute() {

  frc::ChassisSpeeds robotSpeeds = m_drive->GetState().Speeds;
  frc::Rotation2d robotHeading = m_drive->GetState().Pose.Rotation();

  // 通过旋转矩阵，把车身速度变成场地绝对速度
  double vx = robotSpeeds.vx.value() * robotHeading.Cos() - robotSpeeds.vy.value() * robotHeading.Sin();
  double vy = robotSpeeds.vx.value() * robotHeading.Sin() + robotSpeeds.vy.value() * robotHeading.Cos();
  //消除噪声
  if (std::sqrt(vx * vx + vy * vy) < 0.05) {
      vx = 0.0;
      vy = 0.0;
  }
  // 获取底盘当前绝对坐标，并计算对准 Hub 的目标角度
  frc::Pose2d currentPose = m_drive->GetState().Pose;

  double nowX=m_xPID.Calculate(currentPose.X().value(), targetX);
  double nowY=m_yPID.Calculate(currentPose.Y().value(), targetY);
  double speedT=sqrt(nowX*nowX+nowY*nowY);
  if(speedT>MaxSpeed.value()*maxSpeedCoeff){
    double coeff=MaxSpeed.value()*maxSpeedCoeff/speedT;
    nowX*=coeff;
    nowY*=coeff;
  }

  double latencySeconds = 0.45;
  double predictedX = currentPose.X().value() + (nowX * latencySeconds);
  double predictedY = currentPose.Y().value() + (nowY * latencySeconds);
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseX",currentPose.X().value() );
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseY",currentPose.Y().value() );
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseR",robotHeading.Degrees().value() );

  // 使用预测的“未来坐标”来计算对准 Hub 的目标角度和距离
  double dx = m_drive->GetHubPosition().X().value() - predictedX;
  double dy = m_drive->GetHubPosition().Y().value() - predictedY;
  double targetAngleRad = atan2(dy, dx);

  //把底盘即时转换到以机器人与hub的连线为0度的速度向量
  //向前（hub）的速读,依旧使用手柄数据
  double Normvx=nowX*cos(-targetAngleRad)+nowY*cos(PI/2-targetAngleRad);
  //向旁边的速度
  double Normvy=nowX*sin(-targetAngleRad)+nowY*sin(PI/2-targetAngleRad);
  frc::SmartDashboard::PutNumber("shootOnMove/vxToHub",vx );
  frc::SmartDashboard::PutNumber("shootOnMove/vyToHub",vy );
  
  
  //假设射球出膛速度只有真正速度的0.2,need configuration and zone division
  //先横向测试不同距离所需的coeff，然后反求出速度，然后在计算coeff时考虑底盘垂直速度
  double shootCoeff=0.17;
  //shootCoeff=frc::SmartDashboard::GetNumber("shoot_velocity_test", 0.11);
  frc::SmartDashboard::PutNumber("shoot_coeff", shootCoeff);
  //把射球的速度转换成向量
  double Shootvx=m_shooter->vel*cos(m_shooter->Tangle/180.0*PI)*shootCoeff;
  double Shootvz=m_shooter->vel*sin(m_shooter->Tangle/180.0*PI)*shootCoeff;
  //合并向量
  Shootvx=Shootvx-Normvx;
  Normvy=-Normvy;
  double ShooterVelOff=sqrt(Shootvx*Shootvx+Normvy*Normvy+Shootvz*Shootvz)/shootCoeff-m_shooter->vel;
  double chassisAngleOffset=0;
  if(m_shooter->vel>=10)chassisAngleOffset=atan2(Normvy, Shootvx);
  targetAngleRad+=chassisAngleOffset;
  targetAngleRad-=angleOfShooter/180*PI;
  //保证angle不越界
  if(frc::DriverStation::GetAlliance().has_value()&&frc::DriverStation::GetAlliance().value()==frc::DriverStation::Alliance::kRed){
    targetAngleRad-=PI;
    nowX=-nowX;
    nowY=-nowY;
  }
  while(targetAngleRad<-PI)targetAngleRad+=2*PI;
  while(targetAngleRad>PI)targetAngleRad-=2*PI;
  double shootAngleOffset=atan2(Shootvz,Shootvx)/PI*180-m_shooter->Tangle;
  m_shooter->SetAngleOffset(shootAngleOffset);
  m_shooter->SetSpeedOffset(ShooterVelOff);

 
  // 把当前角度和目标角度喂给 PID，算出旋转速度 (omega)
  double currentAngleRad = robotHeading.Radians().value();
  //double omega_rad_per_sec = m_aimPID.Calculate(currentAngleRad, targetAngleRad);

  // frc::SmartDashboard::PutNumber("shootOnMove/xsupplier",m_vXSupplier() );
  units::meters_per_second_t vxt{nowX};
  units::meters_per_second_t vyt{nowY};
  // 下发给底盘
  
  
  frc::Rotation2d rott{units::radian_t(targetAngleRad)};
  m_drive->SetControl(
     driveClosed
          .WithVelocityX(vxt)
          .WithVelocityY(vyt)
          .WithTargetDirection(rott)
  );
  frc::SmartDashboard::PutNumber("shootOnMove/AngleDiff", (targetAngleRad-currentAngleRad)*180/PI);
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVX", Shootvx);
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVY", Normvy);
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVZ", Shootvz);

  bool rightSpeed = std::abs(m_shooter->realShootVelocity - m_shooter->GetShootVelocity()) <= 10.0;
  bool rightRot = m_drive->SOMangleDiff <= 8.0; 
  // 自动路线如果都在可射击区，这里就直接 true。如果有障碍物遮挡区，可以加上 currentPose.Y() 的限制
  bool rightPos = true; 

  if (rightPos && rightSpeed && rightRot) {
    feeder->SetBackwardFeederVelocity(ShooterConstants::kBackwardFeederVelocity);
    feeder->SetUpwardFeederVelocity(ShooterConstants::kUpwardFeederVelocity);
  } else {
    feeder->SetBackwardFeederVelocity(0);
    feeder->SetUpwardFeederVelocity(0);
  }

  frc::Rotation2d ttall{units::radian_t(targetAngleRad)};
  frc::Rotation2d tlow{units::radian_t(currentAngleRad)};
  double angledi=std::abs((ttall-tlow).Degrees().value());
  if(frc::DriverStation::GetAlliance().has_value()&&frc::DriverStation::GetAlliance().value()==frc::DriverStation::Alliance::kRed){
    angledi=180-angledi;
  }
  frc::SmartDashboard::PutNumber("shootOnMove/AngleDiff", angledi);
  m_drive->SOMangleDiff=angledi;
}

// ==========================================
// End：松开按钮瞬间，强制刹车
// ==========================================
void AutoRealTimeAimDrive::End(bool interrupted) {
  //  m_drive->SetControl(
  //    driveClosed
  //         .WithVelocityX(0_mps)
  //         .WithVelocityY(0_mps)
  //         .WithRotationalRate(units::radians_per_second_t{0})
  //);
  feeder->SetBackwardFeederVelocity(0);
    feeder->SetUpwardFeederVelocity(0);
  m_shooter->SetAngleOffset(0);
  m_shooter->SetSpeedOffset(0);
  //停止底盘
    m_drive->SetControl(
        m_drive->Idle
    );
  };


// ==========================================
// IsFinished：配合 WhileTrue 使用，永远不自动结束
// ==========================================
bool AutoRealTimeAimDrive::IsFinished() {
  // 获取当前位置并计算到目标点的直线距离
    frc::Pose2d currentPose = m_drive->GetState().Pose;
    double distance = currentPose.Translation().Distance(m_targetWaypoint.Translation()).value();

    // 当距离小于等于容差（10厘米）时
    return distance <= kTranslationTolerance;
}