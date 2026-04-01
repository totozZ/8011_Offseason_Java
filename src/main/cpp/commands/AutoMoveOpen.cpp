#include "commands/AutoMoveOpen.h"
#include <cmath>
#include <algorithm> // 用于 std::clamp
#include <units/velocity.h>
#include <units/length.h>

using namespace subsystems;

AutoMoveOpen::AutoMoveOpen(CommandSwerveDrivetrain* drivetrain, double targetX, double targetY, double targetHeading, double targetVel,bool invert1, bool invert2)
    : m_drivetrain(drivetrain),  
      targetVelocity(targetVel),
      x(targetX),
      y(targetY),
      d(targetHeading),
      invertA(invert1),
      invertD(invert2) {
    
        driveClosed.WithHeadingPID(8, 0, 0.1)
                    .WithDeadband(MaxSpeed * 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14*1.5})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
    
    // 声明底盘依赖，防止指令冲突
    AddRequirements({m_drivetrain});
    
}
      
void AutoMoveOpen::Initialize() {
    if(invertD){
        d=0.0-d;
        y=8.07-y;
    }
    if(invertA){
        x=16.54-x;
        d=180.0-d;
        while(d>180)d-=360;
        while(d<-180)d+=360;
    }
    auto alliance=frc::DriverStation::GetAlliance();
    double realHead=d;
    if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
        realHead-=180;
    }
    // 指令开始时重置 PID 状态
    m_PID.Reset();
    m_targetWaypoint={units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::degree_t{realHead}}};
    frc::Translation2d nowTrans=m_drivetrain->GetState().Pose.Translation();
    //转换到360坐标吸
    angleToTarget=(m_targetWaypoint.Translation()-nowTrans).Angle().Degrees().value();
    if(angleToTarget<0){
        angleToTarget+=360;
    }
}

void AutoMoveOpen::Execute() {
    // // 1. 获取机器人当前位置
    // frc::Pose2d currentPose = m_drivetrain->GetState().Pose; 
    // // 2. 计算当前位置到目标点的相对向量 (Translation2d)
    // frc::Translation2d difference = m_targetWaypoint.Translation() - currentPose.Translation();
    // double angleNow=difference.Angle().Degrees().value();
    // if(angleNow<0){
    //     angleNow+=360;
    // }
    // //计算角度差
    // angleNow=angleNow-angleToTarget;
    // double supplementAngle=0;
    // //计算补偿向量的方向
    // if(angleNow<0){
    //     supplementAngle=angleToTarget-90;
    // }
    // else{
    //     supplementAngle=angleToTarget+90;
    // }
    // // 当前距离终点的直线距离
    // double distance = difference.Norm().value(); 
    // double diffValue=std::abs(distance*sin(angleNow/180*PI));
    // double suppValue=m_PID.Calculate(0.0,diffValue);
    // double suppX=suppValue*cos(supplementAngle/180*PI);
    // double suppY=suppValue*sin(supplementAngle/180*PI);
    // // 加入默认速度
    // suppX+=targetVelocity*cos(angleToTarget/180*PI);
    // suppY+=targetVelocity*sin(angleToTarget*PI/180);
    // auto alliance=frc::DriverStation::GetAlliance();
    // if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
    //     suppX=-suppX;
    //     suppY=-suppY;

    // }
    // 1. 获取机器人当前位置
    frc::Pose2d currentPose = m_drivetrain->GetState().Pose; 
    
    // 2. 计算当前位置到目标点的相对向量 (Translation2d)
    frc::Translation2d difference = m_targetWaypoint.Translation() - currentPose.Translation();
    
    // 3. 极其优雅：直接获取这个向量的绝对朝向角 (Rotation2d)
    frc::Rotation2d angleTo = difference.Angle();
    
    // 4. 直接把标量速度沿着这个直线角度分解为 X 和 Y 的分量
    // Rotation2d 自带 Cos() 和 Sin()，极其安全且不会有弧度转换的 Bug
    double suppX = targetVelocity * angleTo.Cos();
    double suppY = targetVelocity * angleTo.Sin();
    
    // 5. 保留你的红蓝方反转逻辑
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        suppX = -suppX;
        suppY = -suppY;
    }
    
    // 接下来你就可以直接把 suppX 和 suppY 喂给底盘的 driveClosed / driveOpen 了！
   
    // 5. 应用到底盘
    // FieldCentricFacingAngle 会自动帮你把机器人车头（Rotation）闭环转到 TargetDirection
    m_drivetrain->SetControl(driveClosed
        .WithVelocityX(units::meters_per_second_t{suppX})
        .WithVelocityY(units::meters_per_second_t{suppY})
        .WithTargetDirection(m_targetWaypoint.Rotation()) // 闭环锁定目标点的期望朝向
    );
}

bool AutoMoveOpen::IsFinished() {
    // 获取当前位置并计算到目标点的直线距离
    frc::Pose2d currentPose = m_drivetrain->GetState().Pose;
    double distance = currentPose.Translation().Distance(m_targetWaypoint.Translation()).value();
    frc::Rotation2d angleNow=(m_targetWaypoint.Translation()-currentPose.Translation()).Angle();
    frc::Rotation2d angleT{units::degree_t{angleToTarget}};

    // 当距离小于等于容差（10厘米）时或角度出现大变换（冲过头），指令完成
    return distance <= kTranslationTolerance||std::abs((angleNow-angleT).Degrees().value())>=80;
}

void AutoMoveOpen::End(bool interrupted) {
    auto currentSpeeds = m_drivetrain->GetState().Speeds;

    // 把这些速度赋给底盘身上那个绝对安全的滑行 Request
    if(interrupted){
        m_drivetrain->SetControl(
        m_drivetrain->Idle );
    }
    else
    m_drivetrain->SetControl(
        m_drivetrain->m_safeCoastRequest
            .WithVelocityX(currentSpeeds.vx)
            .WithVelocityY(currentSpeeds.vy)
            .WithRotationalRate(currentSpeeds.omega)
    );
}