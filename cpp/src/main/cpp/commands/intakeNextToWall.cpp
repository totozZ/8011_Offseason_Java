#include "commands/intakeNextToWall.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

intakeNextToWall::intakeNextToWall(CommandSwerveDrivetrain* drive,
    GroundIntakeSubsystem* g, frc2::CommandXboxController* j, bool oppo)
  : joy(j), m_drive(drive), m_ground(g), opposite(oppo) {
  
  AddRequirements({drive}); 
  AddRequirements({g});
}

void intakeNextToWall::Initialize() {
    m_movePIDX.Reset();
    m_movePIDY.Reset();
    isRed = false;
    arrivedAtFirstPoint = false;
    arrivedAt0=false;
    finishedAll = false;

    driveClosed.WithHeadingPID(8, 0, 0.1)
               .WithDeadband(MaxSpeed * 0.05)
               .WithRotationalDeadband(units::radians_per_second_t{0.1})
               .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14})
               .WithDriveRequestType(swerve::DriveRequestType::Velocity)
               .WithSteerRequestType(swerve::SteerRequestType::Position);

    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        isRed = true;
    }
    else{
        isRed=false;
    }
    bool sym=isRed;
    if(opposite){
        sym=!isRed;
    }
    double currentY=m_drive->GetState().Pose.Y().value();
    if(sym){
        currentY=FieldConstants::kFieldWidth.value()-currentY;
    }
    if(currentY<3.7){
        targetX_0=0.6;
        targetY_0=0.6;
        targetR_0=-135;
        targetX_1=0.6;
        targetY_1=0.5;
        targetR_1=110;
        targetX_2=0.45;
        targetY_2=2.7;
        targetR_2=110;
        
    }
    else{
        targetX_0=3;
        targetY_0=7.25;
        targetR_0=135;
        targetX_1=0.75;
        targetY_1=7.5;
        targetR_1=-110;
        targetX_2=0.45;
        targetY_2=5;
        targetR_2=-110;
    }

    if(sym){
        targetX_0=FieldConstants::kFieldLength.value()-targetX_0;
        targetY_0=FieldConstants::kFieldWidth.value()-targetY_0;
        if(!isRed) targetR_0-=180;
        targetX_1=FieldConstants::kFieldLength.value()-targetX_1;
        targetY_1=FieldConstants::kFieldWidth.value()-targetY_1;
        if(!isRed) targetR_1-=180;
        targetX_2=FieldConstants::kFieldLength.value()-targetX_2;
        targetY_2=FieldConstants::kFieldWidth.value()-targetY_2;
        if(!isRed)targetR_2-=180;
    }
    if(opposite&&isRed){
        targetR_0-=180;
        targetR_1-=180;
        targetR_2-=180;
    }
    double diffX=m_drive->GetState().Pose.X().value()-targetX_1;
    double diffY=m_drive->GetState().Pose.Y().value()-targetY_1;
    stopRightAway=std::sqrt(diffX*diffX+diffY*diffY)>=10;
    m_ground->SetPitchNormPosition(GroundIntakeConstants::PitchNormPosition);
    m_ground->SetRollerVelocity(100.0);
}

void intakeNextToWall::Execute() {
    frc::Pose2d currentPose = m_drive->GetState().Pose;
    double currentX = currentPose.X().value();
    double currentY = currentPose.Y().value();

    double speedX = 0;
    double speedY = 0;
    
    // if(!arrivedAt0){
    //     double disTo0 = std::sqrt(std::pow(currentX - targetX_0, 2) + std::pow(currentY - targetY_0, 2));
    //     targetSpeed=1.0;
    //     speedX = m_movePIDX.Calculate(currentX, targetX_0);
    //     speedY = m_movePIDY.Calculate(currentY, targetY_0);
    //     if(disTo0<0.5){
    //         // 开启地吸 (你可以根据 disToSecond 的距离决定何时开启)
    //     m_ground->SetPitchNormPosition(0.99);
    //     m_ground->SetRollerDutyCycle(0.6);
    //     }

    //     // 如果距离第一个点足够近（例如小于 0.3 米），切换状态
    //     if (disTo0 < 0.1) {
    //         arrivedAt0 = true;
    //         // 可以在这里重置 PID，为了第二段移动更平滑
    //         m_movePIDX.Reset();
    //         m_movePIDY.Reset();
    //     }
    //     targetDire=targetR_0;
    // }
    if (!arrivedAtFirstPoint) {
        double disToFirst = std::sqrt(std::pow(currentX - targetX_1, 2) + std::pow(currentY - targetY_1, 2));
        targetSpeed=2.2;
        speedX = m_movePIDX.Calculate(currentX, targetX_1);
        speedY = m_movePIDY.Calculate(currentY, targetY_1);
        // if(disToFirst<1){
        //     // 开启地吸 (你可以根据 disToSecond 的距离决定何时开启)
        // m_ground->SetPitchNormPosition(0.99);
        // m_ground->SetRollerDutyCycle(0.6);
        // }
        // 如果距离第一个点足够近（例如小于 0.3 米），切换状态
        if (disToFirst < 0.1) {
            arrivedAtFirstPoint = true;
            // 可以在这里重置 PID，为了第二段移动更平滑
            m_movePIDX.Reset();
            m_movePIDY.Reset();
        }
        targetDire=targetR_1;

        double speedT = std::sqrt(speedX * speedX + speedY * speedY);
        if (speedT >= targetSpeed) {
            double coeff = speedT / targetSpeed;
            speedX /= coeff;
            speedY /= coeff;
        }
    } 
    else {
        targetSpeed=1.5;
        speedX =  -joy->GetLeftY()*TunerConstants::kSpeedAt12Volts.value()*0.2;
        
        speedY = m_movePIDY.Calculate(currentY, targetY_2);
        speedY=std::clamp(speedY,-targetSpeed,targetSpeed);
        if(isRed){
            speedX=-speedX;
        }
        if (targetY_1 < targetY_2) {
            finishedAll = (currentY >= targetY_2);
        } else {
            finishedAll = (currentY <= targetY_2);
        }
        targetDire=targetR_2;
    }

    // === 处理联盟镜像 ===
    if (isRed) {
        speedX = -speedX;
        speedY = -speedY;
        
    } 

    // === 发送底盘指令 ===
    m_drive->SetControl(driveClosed
        .WithVelocityX(units::meters_per_second_t(speedX))
        .WithVelocityY(units::meters_per_second_t(speedY))
        .WithTargetDirection(frc::Rotation2d{units::degree_t(targetDire)})
    );

    // 调试输出
    frc::SmartDashboard::PutBoolean("arrivedAtFirstPoint", arrivedAtFirstPoint);
    frc::SmartDashboard::PutBoolean("finishedAll", finishedAll);
}

void intakeNextToWall::End(bool interrupted) {
    
    //m_ground->SetPitchNormPosition(0.99);
    m_ground->Stop();
    auto currentSpeeds =m_drive->GetState().Speeds;
    m_drive->SetControl(
        m_drive->m_safeCoastRequest
            .WithVelocityX(currentSpeeds.vx)
            .WithVelocityY(currentSpeeds.vy)
            .WithRotationalRate(currentSpeeds.omega)
    );
}

bool intakeNextToWall::IsFinished() {
    // 根据你的需求定义结束条件
    return finishedAll||stopRightAway; 
}
