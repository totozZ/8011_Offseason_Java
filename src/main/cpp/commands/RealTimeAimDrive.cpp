#include "commands/RealTimeAimDrive.h"
#include <numbers> 

RealTimeAimDrive::RealTimeAimDrive(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, std::function<double()> vx, std::function<double()> vy,double AOSDeg)
    : m_drive(drive), m_shooter(sh), m_vXSupplier(vx), m_vYSupplier(vy),angleOfShooter(AOSDeg){
  
  AddRequirements({drive}); // 声明占用底盘
  
  // 告诉 PID -180度和180度是相连的
  //m_aimPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

// ==========================================
// Initialize：按下按钮的瞬间清空旧状态
// ==========================================
void RealTimeAimDrive::Initialize() {
  //m_aimPID.Reset(); 
  driveClosed.WithHeadingPID(8, 0, 0.1)
                    .WithDeadband(MaxSpeed * 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})        
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14*1.2})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
  //m_drive->changeDriveCurrentLimit(20.0);
}

void RealTimeAimDrive::Execute() {

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
  
  frc::Pose2d currentPose = m_drive->GetState().Pose;
  double curX = currentPose.X().value();
  double curY = currentPose.Y().value();
  double hubX = m_drive->GetHubPosition().X().value();
  double hubY = m_drive->GetHubPosition().Y().value();

  double dx_current = hubX - curX;
  double dy_current = hubY - curY;
  // 当前真实的 Hub 角度 
  double currentAngleToHubRad = std::atan2(dy_current, dx_current); 

  // 获取手柄输入的预期场地方向速度
  double nowX = frc::ApplyDeadband(m_vXSupplier(), 0.05) * MaxSpeed.value() * 0.3;
  double nowY = frc::ApplyDeadband(m_vYSupplier(), 0.05) * MaxSpeed.value() * 0.3;
  
  // 注意红方时需要反转
  double realX = 0;
  double realY = 0;
  if (frc::DriverStation::GetAlliance().has_value() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
    realX = -nowX;
    realY = -nowY;
  } else {
    realX = nowX;
    realY = nowY;
  }

  // v_radial: 径向速度 (正代表靠近 Hub，负代表远离)  
  double v_radial = realX * cos(currentAngleToHubRad) + realY * sin(currentAngleToHubRad);
  
  // v_tangential: 切向/横向速度 (正代表逆时针绕 Hub 走)
  // 相当于把场地方向旋转 -currentAngleToHubRad
  double v_tangential = -realX * sin(currentAngleToHubRad) + realY * cos(currentAngleToHubRad);

  double latencyRadialSeconds = 0.3;     // 竖向（靠近/远离）的预测时间
  double latencyTangentialSeconds = -0.1; // 横向（绕圈）的预测时间

  double d_radial = v_radial * latencyRadialSeconds;
  double d_tangential = v_tangential * latencyTangentialSeconds;

  double deltaX = d_radial * cos(currentAngleToHubRad) - d_tangential * sin(currentAngleToHubRad);
  double deltaY = d_radial * sin(currentAngleToHubRad) + d_tangential * cos(currentAngleToHubRad);

  double predictedX = curX + deltaX;
  double predictedY = curY + deltaY;

  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseX", curX);
  frc::SmartDashboard::PutNumber("shootOnMove/CurrentPoseY", curY);
  frc::SmartDashboard::PutNumber("shootOnMove/PredictedX", predictedX);
  frc::SmartDashboard::PutNumber("shootOnMove/PredictedY", predictedY);

  double dx_pred = hubX - predictedX;
  double dy_pred = hubY - predictedY;
  double targetAngleRad = std::atan2(dy_pred, dx_pred);

  //把底盘即时转换到以机器人与hub的连线为0度的速度向量
  //向前（hub）的速读,依旧使用手柄数据
  double Normvx=realX*cos(-targetAngleRad)+realY*cos(PI/2-targetAngleRad);
  //向旁边的速度
  double Normvy=realX*sin(-targetAngleRad)+realY*sin(PI/2-targetAngleRad);
  // frc::SmartDashboard::PutNumber("shootOnMove/vxToHub",vx );
  // frc::SmartDashboard::PutNumber("shootOnMove/vyToHub",vy );
  
  
  //假设射球出膛速度只有真正速度的0.2,need configuration and zone division
  //先横向测试不同距离所需的coeff，然后反求出速度，然后在计算coeff时考虑底盘垂直速度
  double shootCoeff=0.25;
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
  if(frc::DriverStation::GetAlliance().has_value()&&frc::DriverStation::GetAlliance().value()==frc::DriverStation::Alliance::kRed){
    targetAngleRad-=PI;
  }
  double shootAngleOffset=atan2(Shootvz,Shootvx)/PI*180-m_shooter->Tangle;
  m_shooter->SetAngleOffset(shootAngleOffset);
  m_shooter->SetSpeedOffset(ShooterVelOff);

  double currentAngleRad = robotHeading.Radians().value();
  //double omega_rad_per_sec = m_aimPID.Calculate(currentAngleRad, targetAngleRad);


  // frc::SmartDashboard::PutNumber("shootOnMove/xsupplier",m_vXSupplier() );
  units::meters_per_second_t vxt{nowX};
  units::meters_per_second_t vyt{nowY};
  
  frc::Rotation2d rott{units::radian_t(targetAngleRad)};
  frc::Rotation2d ttall{units::radian_t(targetAngleRad)};
  frc::Rotation2d tlow{units::radian_t(currentAngleRad)};
  double angledi=std::abs((ttall-tlow).Degrees().value());
  
  if(frc::DriverStation::GetAlliance().has_value()&&frc::DriverStation::GetAlliance().value()==frc::DriverStation::Alliance::kRed){
    angledi=180-angledi;
  }

  // 下发给底盘：加入刹车判定
  if (std::abs(m_vXSupplier()) < 0.1 && std::abs(m_vYSupplier()) < 0.1 && angledi < 2.0) {
      m_drive->SetControl(driveBrake);
  } else {
      m_drive->SetControl(
          driveClosed
              .WithVelocityX(vxt)
              .WithVelocityY(vyt)
              .WithTargetDirection(rott)
      );
  }

  frc::SmartDashboard::PutNumber("shootOnMove/AngleDiff", angledi);
  m_drive->SOMangleDiff=angledi;
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVX", Shootvx);
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVY", Normvy);
  frc::SmartDashboard::PutNumber("shootOnMove/ShootVZ", Shootvz);
}

// ==========================================
// End：松开按钮瞬间，强制刹车
// ==========================================
void RealTimeAimDrive::End(bool interrupted) {
  //  m_drive->SetControl(
  //    driveClosed
  //         .WithVelocityX(0_mps)
  //         .WithVelocityY(0_mps)
  //         .WithRotationalRate(units::radians_per_second_t{0})
  //);
 // m_drive->changeDriveCurrentLimit(40.0);
  m_shooter->SetAngleOffset(0);
  m_shooter->SetSpeedOffset(0);
  auto currentSpeeds = m_drive->GetState().Speeds;

    // 把这些速度赋给底盘身上那个绝对安全的滑行 Request
    m_drive->SetControl(
        m_drive->m_safeCoastRequest
            .WithVelocityX(currentSpeeds.vx)
            .WithVelocityY(currentSpeeds.vy)
            .WithRotationalRate(currentSpeeds.omega)
    );
  };


// ==========================================
// IsFinished：配合 WhileTrue 使用，永远不自动结束
// ==========================================
bool RealTimeAimDrive::IsFinished() {
  return false; 
}