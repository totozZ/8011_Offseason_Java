#include "commands/intakeNextToHub.h"
#include <numbers>

intakeNextToHub::intakeNextToHub(CommandSwerveDrivetrain* drive, ShooterSubsystem* sh, GroundIntakeSubsystem* g,frc2::CommandXboxController* j, bool oppo)
  : m_drive(drive), m_shooter(sh), m_ground(g), joy(j),opposite(oppo){
  
  AddRequirements({drive}); // 声明占用底盘
  AddRequirements({g});//占用地吸
  //m_aimPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

void intakeNextToHub::Initialize(){
    m_movePIDX.Reset();
    isRed=false;
      m_movePIDY.Reset();
    driveClosed.WithHeadingPID(8, 0, 0.1)
                    .WithDeadband(MaxSpeed * 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
    startY=m_drive->GetState().Pose.Y().value();
    auto alliance=frc::DriverStation::GetAlliance();

    if(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed){
        targetX=10.55; 
        isRed=true;
    }
    else{
        targetX=5.6; 
    }
    if(opposite){
      targetX=16.54-targetX;
    }

    if(startY<=4.035){//离右边近，从右边吸
        targetY=2.0; 
        rawTargetRotation=90;
        speedYDef=2.0;
      }
      else {
        targetY=6.0;
        rawTargetRotation=-90;
        speedYDef=-2.0;
      }
    if(targetX<=8.27){
      rawTargetRotation=180-rawTargetRotation;
    }
    arrived=false;
    m_ground->SetPitchNormPosition(GroundIntakeConstants::PitchNormPosition);
    m_ground->SetRollerVelocity(100.0);
        
}

void intakeNextToHub::Execute(){
      frc::Pose2d currentPose = m_drive->GetState().Pose;

      double currentX=currentPose.X().value();
      double currentY=currentPose.Y().value();
      stopRightAway=currentX<=4.0||currentX>=12.5;
      double speedX=0;
      double speedY=0;
      //无论如何先开到一边后再吸
      double dis=sqrt((currentX-targetX)*(currentX-targetX)+(currentY-targetY)*(currentY-targetY));

      if(!arrived){
        speedX=m_movePIDX.Calculate(currentX, targetX);
        speedY=m_movePIDY.Calculate(currentY, targetY);
        double speedT=sqrt(speedX*speedX+speedY*speedY);
        double maxSpeed=2.4;
        if(speedT>=maxSpeed){
          double coeff=speedT/maxSpeed;
          speedX/=coeff;
          speedY/=coeff;
        }
      }
      //抵达了，直接开吸
      else{
        if(joy==nullptr){//自动阶段给nullptr
          speedX=0;
        }
        else{
          speedX=-joy->GetLeftY()*TunerConstants::kSpeedAt12Volts.value()*0.2;
        }
        if(isRed){
          speedX=-speedX;
        }
        speedY=speedYDef;
        // m_ground->SetPitchNormPosition(0.99);
        // m_ground->SetRollerDutyCycle(0.6);
      }
     
      if(isRed){
        speedX=-speedX;
        speedY=-speedY;
        targetDire=rawTargetRotation-180;
      }
      else{
        targetDire=rawTargetRotation;
      }
      frc::Rotation2d rott{units::degree_t(targetDire)};
      m_drive->SetControl(driveClosed
        .WithVelocityX(units::meters_per_second_t(speedX))
        .WithVelocityY(units::meters_per_second_t(speedY))
        .WithTargetDirection(rott)
    );

    double angleDiff=targetDire-m_drive->GetState().Pose.Rotation().Degrees().value();
    while(angleDiff>180){angleDiff-=360;}
    while(angleDiff<-180){angleDiff+=360;}
    angleDiff=std::abs(angleDiff);
    bool rota=false;
    if(isRed){
      rota=angleDiff>=160;
    }
    else{
      rota=angleDiff<=20;
    }
    if(dis<0.1&&rota)
    arrived=true;
    frc::SmartDashboard::PutBoolean("arrived", arrived);
    frc::SmartDashboard::PutNumber("aDis", dis);
    frc::SmartDashboard::PutNumber("aAngleDiff", angleDiff);
      
}

void intakeNextToHub::End(bool interrupted){
   m_ground->Stop();
   auto currentSpeeds =m_drive->GetState().Speeds;

    // 把这些速度赋给底盘身上那个绝对安全的滑行 Request
    m_drive->SetControl(
        m_drive->m_safeCoastRequest
            .WithVelocityX(currentSpeeds.vx)
            .WithVelocityY(currentSpeeds.vy)
            .WithRotationalRate(currentSpeeds.omega)
    );
}

bool intakeNextToHub::IsFinished(){

    double currentY=m_drive->GetState().Pose.Y().value();
    return stopRightAway||(arrived&&(currentY<=1||currentY>=7));
}