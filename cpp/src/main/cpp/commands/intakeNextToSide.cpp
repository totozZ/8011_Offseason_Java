#include "commands/intakeNextToSide.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

intakeNextToSide::intakeNextToSide(CommandSwerveDrivetrain* drive,
    GroundIntakeSubsystem* g, frc2::CommandXboxController* joy, bool oppo)
  : m_drive(drive), m_ground(g), joystick(joy), opposite(oppo) {
  
  AddRequirements({drive}); 
  AddRequirements({g});
}

void intakeNextToSide::Initialize() {
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
    double currentY=m_drive->GetState().Pose.Y().value();
    double currentX=m_drive->GetState().Pose.X().value();
    if(currentX<11.54&&currentX>5){
        targetX_1=10.54;
        targetX_2=6;
        targetR_1=180;
        targetR_2=180;
        if(currentX<8.27){
        targetX_1=FieldConstants::kFieldLength.value()-targetX_1;
        targetY_1=FieldConstants::kFieldWidth.value()-targetY_1;
        targetX_2=FieldConstants::kFieldLength.value()-targetX_2;
        targetY_2=FieldConstants::kFieldWidth.value()-targetY_2;
        targetR_1=00;
        targetR_2=00;
        }
    }
    else if(currentX<5){
        targetX_1=3.5;
        targetX_2=0.55;
        targetR_1=180;
        targetR_2=180;
        if(currentX<2){
            double a =targetX_1;
            targetX_1=targetX_2;
            targetX_2=a;
            targetR_1=00;
            targetR_2=00;
        }
    }
    else{
        targetX_1=FieldConstants::kFieldLength.value()-3.5;
        targetX_2=FieldConstants::kFieldLength.value()-0.55;
        targetR_1=00;
        targetR_2=00;
        if(currentX>14.56){
            double a =targetX_1;
            targetX_1=targetX_2;
            targetX_2=a;
            targetR_1=180;
            targetR_2=180;
        }
    }
    if(currentY<4.035){
        targetY_1=0.60;
        targetY_2=0.60;
        targetR_1=-targetR_1;
        targetR_2=-targetR_2;
    }
    else{
        targetY_1=FieldConstants::kFieldWidth.value()-0.60;
        targetY_2=FieldConstants::kFieldWidth.value()-0.60;
    }
    double diffX=m_drive->GetState().Pose.X().value()-targetX_1;
    double diffY=m_drive->GetState().Pose.Y().value()-targetY_1;
    stopRightAway=std::sqrt(diffX*diffX+diffY*diffY)>=10;
    m_ground->SetPitchNormPosition(GroundIntakeConstants::PitchNormPosition);
    m_ground->SetRollerVelocity(100.0);
}

void intakeNextToSide::Execute() {
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
        // if(disToFirst<0.5){
        //     // 开启地吸 (你可以根据 disToSecond 的距离决定何时开启)
        // m_ground->SetPitchNormPosition(0.99);
        // m_ground->SetRollerDutyCycle(0.6);
        // }
        // 如果距离第一个点足够近（例如小于 0.3 米），切换状态
        if (disToFirst < 0.1) {
            arrivedAtFirstPoint = true;
           
            m_movePIDX.Reset();
            m_movePIDY.Reset();
        }
        targetDire=targetR_1;
        speedT = std::sqrt(speedX * speedX + speedY * speedY);
        if(speedT>1e-6&&speedT<speedLowerLimit){
            double coeff = speedT / speedLowerLimit;
            speedX /= coeff;
            speedY /= coeff;
        }
        if (speedT >= targetSpeed) {
            double coeff = speedT / targetSpeed;
            speedX /= coeff;
            speedY /= coeff;
        }
    } 
    // === 第二阶段：开往最终墙边点并吸球 ===
    else {
        targetSpeed=2.0;
        speedX = m_movePIDX.Calculate(currentX, targetX_2);
        speedX=std::clamp(speedX,-targetSpeed,targetSpeed);
        speedY = -joystick->GetLeftX()*TunerConstants::kSpeedAt12Volts.value()*0.2;
        if(isRed){
            speedY=-speedY;
        }
        // 如果到达最终点
        bool goForward= targetR_2<=90&&targetR_2>=-90;
        finishedAll = (goForward&&currentX>targetX_2)||(!goForward&&currentX<targetX_2); 
        
        targetDire=targetR_2;
    }

    // === 处理联盟镜像 ===
    if (isRed) {
        speedX = -speedX;
        speedY = -speedY;
        targetDire-=180;
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

void intakeNextToSide::End(bool interrupted) {
    
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

bool intakeNextToSide::IsFinished() {
    // 根据你的需求定义结束条件
    return finishedAll; 
}
