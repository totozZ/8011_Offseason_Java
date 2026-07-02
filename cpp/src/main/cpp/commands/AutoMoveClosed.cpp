#include "commands/AutoMoveClosed.h"
#include <cmath>
#include <algorithm> // 用于 std::clamp
#include <units/velocity.h>
#include <units/length.h>

using namespace subsystems;

AutoMoveClosed::AutoMoveClosed(CommandSwerveDrivetrain* drivetrain, double targetX, double targetY, double targetHeading, double targetVel, bool invert1, bool invert2)
    : invertD(invert2),
      invertA(invert1),
      x(targetX),
      y(targetY),
      d(targetHeading),
      m_drivetrain(drivetrain),
      targetVelocity(targetVel) {
    
        driveClosed.WithHeadingPID(8, 0, 0.1)
                    .WithDeadband(MaxSpeed * 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14*1.5})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
    
    // 声明底盘依赖，防止指令冲突
    AddRequirements({m_drivetrain});
    
}
      
void AutoMoveClosed::Initialize() {
    if(invertD){
        d=0.0-d;
        y=8.07-y;
    }
    if(invertA){
        x=FieldConstants::kFieldLength.value()-x;
        d=180.0-d;
        while(d>180)d-=360;
        while(d<-180)d+=360;
    }
    auto alliance=frc::DriverStation::GetAlliance();
    realHead=d;
    if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
        realHead-=180;
    }
    m_targetWaypoint={units::meter_t{x},units::meter_t{y}, frc::Rotation2d{units::degree_t{realHead}}};
    m_MovePIDx.Reset();
    m_MovePIDy.Reset();
}

void AutoMoveClosed::Execute() {
    // 1. 获取机器人当前位置
    frc::Pose2d currentPose = m_drivetrain->GetState().Pose; 
    // 2. 计算当前位置到目标点的相对向量 (Translation2d)
    double currentX=currentPose.X().value();
    double currentY=currentPose.Y().value();
    double speedX=m_MovePIDx.Calculate(currentX, x);
    double speedY=m_MovePIDy.Calculate(currentY, y);
    double speedT=sqrt(speedX*speedX+speedY*speedY);
      
      if(speedT>=targetVelocity){
        double coeff=speedT/targetVelocity;
        speedX/=coeff;
        speedY/=coeff;
      }
    auto alliance=frc::DriverStation::GetAlliance();
    if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
        speedX=-speedX;
        speedY=-speedY;
    }
    // 5. 应用到底盘
    // FieldCentricFacingAngle 会自动帮你把机器人车头（Rotation）闭环转到 TargetDirection
    if(m_drivetrain->autoShooting){
        targetRot=m_drivetrain->autoRot;
    }
    else{
        targetRot=realHead;
    }
    m_drivetrain->SetControl(driveClosed
        .WithVelocityX(units::meters_per_second_t{speedX})
        .WithVelocityY(units::meters_per_second_t{speedY})
        .WithTargetDirection(frc::Rotation2d{units::degree_t{targetRot}}) // 闭环锁定目标点的期望朝向
    );
}

bool AutoMoveClosed::IsFinished() {
    // 获取当前位置并计算到目标点的直线距离
    frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
    double distance = currentPose.Translation().Distance(m_targetWaypoint.Translation()).value();
    frc::Rotation2d realT(units::degree_t{targetRot});
    double Adiff=(currentPose.Rotation()-realT).Degrees().value();
    Adiff=std::abs(Adiff);
    auto alliance=frc::DriverStation::GetAlliance();
    if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
        Adiff=180-Adiff;
    }
    frc::SmartDashboard::PutNumber("Adiff", Adiff);
    // 当距离小于等于容差（10厘米）时
    return distance <= kTranslationTolerance&&Adiff<5;
}

void AutoMoveClosed::End(bool interrupted) {
    m_drivetrain->SetControl(m_drivetrain->Idle); 

}
