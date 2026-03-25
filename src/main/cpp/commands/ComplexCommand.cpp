#include "commands/ComplexCommand.h"

#include <frc2/command/Commands.h>
#include <frc2/command/DeferredCommand.h>
#include <wpi/SmallSet.h>

ComplexCommand::ComplexCommand(
    subsystems::CommandSwerveDrivetrain* driveSubsystem,
    subsystems::VisionSubsystem* visionSubsystem,
    subsystems::ShooterSubsystem* shooterSubsystem,
    subsystems::FeederSubsystem* feederSubsystem,
    subsystems::GroundIntakeSubsystem* groundIntakeSubsystem)
    : m_drivesubsystem(driveSubsystem),
      m_visionSubsystem(visionSubsystem),
      m_shooterSubsystem(shooterSubsystem),
      m_feederSubsystem(feederSubsystem),
      m_groundIntakeSubsystem(groundIntakeSubsystem)
    {}

frc2::CommandPtr ComplexCommand::FollowPathCommand(frc::Pose2d targetPos) {
  return frc2::cmd::DeferredProxy([this, targetPos]() {
                return m_drivesubsystem->followPathCommand(
                    targetPos);
              });
}

//和pov bind
frc2::CommandPtr ComplexCommand::MoveOnShoot(std::function<int()> supplier) {
  return frc2::cmd::Defer([this, supplier]() {
                int a=supplier();
                return m_drivesubsystem->followShootOnMovePathCommand(a);
              },
            {m_drivesubsystem});
}

frc2::CommandPtr ComplexCommand::GroundintakeprepareCommand() {
  return frc2::cmd::Sequence(
          m_groundIntakeSubsystem->SetPitchNormPositionCommandPtr(0.93),
          //m_groundIntakeSubsystem->SetRollerVelocityCommandPtr(80.0),
          m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.1)
        );
}

frc2::CommandPtr ComplexCommand::GroundintakeassistCommand() {
  return frc2::cmd::Sequence(
      frc2::cmd::Wait(units::second_t{0.7}),
      m_groundIntakeSubsystem->SetPitchNormPositionCommandPtr(0.05),
      //m_groundIntakeSubsystem->SetRollerVelocityCommandPtr(20),
      frc2::cmd::Wait(units::second_t{0.7}),
      m_groundIntakeSubsystem->SetPitchNormPositionCommandPtr(0.85)
      
  );
}

frc2::CommandPtr ComplexCommand::GroundintakeresetCommand() {
  return frc2::cmd::Sequence(
      m_groundIntakeSubsystem->StopCommandPtr(),
          m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0)
    );
}

frc2::CommandPtr ComplexCommand::PreloadCommand() {
  static constexpr double kPreloadCurrentThresholdA = 12.5;
  static constexpr units::second_t kPostThresholdDelay = units::second_t{0.6};

  return frc2::cmd::Sequence(
      m_feederSubsystem->SetUpwardFeederCurrentCommandPtr(13.0, 0.25),
      m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.3),
      frc2::cmd::WaitUntil([this] {
        return m_feederSubsystem->GetUpwardFeederCurrent() >
               kPreloadCurrentThresholdA;
      }),
      frc2::cmd::Wait(kPostThresholdDelay),
      m_feederSubsystem->SetUpwardFeederCurrentCommandPtr(0.0, 0.25),
      m_feederSubsystem->SetBackwardFeederDutyCommandPtr(0.1));
}

frc2::CommandPtr ComplexCommand::ShootWithFeederCommand() {
  return frc2::cmd::Parallel(
      //m_shooterSubsystem->HoldShootVelocityCommandPtr(
        //  ShooterConstants::kShootVelocity),
        m_shooterSubsystem->EnableShooter(),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this]{return 
            std::abs(m_shooterSubsystem->GetShootVelocity()-m_shooterSubsystem->realShootVelocity)<0.5
            &&m_drivesubsystem->SOMangleDiff<=5;})
          .WithTimeout(units::second_t{1.5}),
          m_feederSubsystem->setdutyCommandPtr(0.8, 0.8)
              ));
}

frc2::CommandPtr ComplexCommand::assistPassing(){
  return frc2::cmd::Parallel(
      //m_shooterSubsystem->HoldShootVelocityCommandPtr(
        //  ShooterConstants::kShootVelocity),
        m_shooterSubsystem->EnableShooter(),
      frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([this]{return std::abs(m_shooterSubsystem->GetShootVelocity()-m_shooterSubsystem->realShootVelocity)<1;})
          .WithTimeout(units::second_t{2}),
          m_feederSubsystem->setdutyCommandPtr(0.8, 0.8),
              frc2::cmd::Sequence(
                frc2::cmd::Wait(units::second_t{1}),
                ComplexCommand::GroundintakeassistCommand()).Repeatedly()
              
              ));
}

frc2::CommandPtr ComplexCommand::StopShootWithFeederCommand() {
  return frc2::cmd::Sequence(m_shooterSubsystem->DisableShooter(),
                             m_feederSubsystem->StopCommandPtr());
}

frc2::CommandPtr ComplexCommand::autoFollow(frc::Pose2d targetPos) {
  return FollowPathCommand(targetPos);
}

frc2::CommandPtr ComplexCommand::FollowAndShootCommand(frc::Pose2d targetPos) {
  return frc2::cmd::Sequence(
      FollowPathCommand(targetPos));
}

frc2::CommandPtr ComplexCommand::FollowAndShootCommand2(frc::Pose2d targetPos) {
  return frc2::cmd::Parallel(
      FollowPathCommand(targetPos),
      ShootWithFeederCommand()
    );
}

frc2::CommandPtr ComplexCommand::PassBump(bool atOppo) {
  
  // 提前把底盘指针拿出来，避免在 Lambda 里使用 this
  auto drive = m_drivesubsystem;

  // 使用最清爽的 DeferredProxy
  return frc2::cmd::DeferredProxy([drive,atOppo]() {
    double innerX = 3.43;
    double innerY = 5.6;
    double innerR = 45; 
    double OuterX = 6;
    double OuterY = 5.6;
    double OuterR = 45;

    auto alliance = frc::DriverStation::GetAlliance();
    
    // 全部使用局部的 drive 指针，断绝一切内存生命周期烦恼
    double x = drive->GetState().Pose.X().value();
    double y = drive->GetState().Pose.Y().value(); 

    double targetSpeed = 2;
    bool invertD = y <= 4;
    bool invertA = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
    if(atOppo){
      invertA=!invertA;
    }
    // 根据实时 X 坐标决定先后顺序
    if (x > 5 && x < 11.54) {
      innerX-=0.5;
      innerR=135;
      OuterR=135;
      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, OuterX, OuterY, OuterR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveClosed(drive, innerX, innerY, innerR, targetSpeed-0.4, invertA, invertD).ToPtr()
      ); 
    } else {
      OuterX+=0.5;
      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, innerX, innerY, innerR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveClosed(drive, OuterX, OuterY, OuterR, targetSpeed-0.4, invertA, invertD).ToPtr()
      ); 
    }
  });
}

#include <cmath>

frc2::CommandPtr ComplexCommand::PassTrench(bool atOppo) {
  
  auto drive = m_drivesubsystem;

  return frc2::cmd::DeferredProxy([drive, atOppo]() {
    // 初始基准点
    double innerX = 3.2;
    double innerY = 7.4;
    double innerR = 0; 
    double OuterX = 5.9;
    double OuterY = 7.4;
    double OuterR = 0;

    auto alliance = frc::DriverStation::GetAlliance();
    
    double x = drive->GetState().Pose.X().value();
    double y = drive->GetState().Pose.Y().value(); 

    double targetSpeed = 2.4;
    bool invertD = y <= 4;
    bool invertA = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
    if(atOppo){
      invertA = !invertA;
    }

    // 将机器人的实际坐标反向投射到基准坐标系
    double virtualX = invertA ? (16.54 - x) : x;
    double virtualY = invertD ? (8.07 - y) : y;

    double offsetDist = 0.3; 

    // 根据实时 X 坐标决定先后顺序
    if (x > 5 && x < 11.54) {
      
      // 1. 先算出从当前位置指向 Outer 点的绝对弧度
      double angleRad = std::atan2(OuterY - virtualY, OuterX - virtualX);
      
      // 2. 核心数学：将 Outer 点沿着这个角度“反向”移动 0.3 米
      OuterX -= offsetDist * std::cos(angleRad);
      OuterY -= offsetDist * std::sin(angleRad);
      
      // 3. 将弧度转回度数发给底盘
      OuterR = 180;
      innerR = 180; 

      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, OuterX, OuterY, OuterR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveCircle(drive,OuterX-0.4,innerY-0.4,0.2,90,true,targetSpeed,false,180,false,invertA,invertD,0).ToPtr(),
        AutoMoveClosed(drive, innerX, innerY, innerR, targetSpeed, invertA, invertD).ToPtr()
      ); 
    } else {
      
      // 1. 算出从当前位置指向 inner 点的绝对弧度
      double angleRad = std::atan2(innerY - virtualY, innerX - virtualX);
      
      // 2. 核心数学：将 inner 点沿着这个角度“反向”移动 0.3 米
      innerX -= offsetDist * std::cos(angleRad);
      innerY -= offsetDist * std::sin(angleRad);
      
      // 3. 将弧度转回度数发给底盘
      innerR = 0;
      OuterR = 0; 

      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, innerX, innerY, innerR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveCircle(drive,innerX+0.4,OuterY-0.4,0.2,90,false,targetSpeed,false,0,false,invertA,invertD,0).ToPtr(),
        AutoMoveClosed(drive, OuterX, OuterY, OuterR, targetSpeed, invertA, invertD).ToPtr()
      ); 
    }
  });
}
frc2::CommandPtr ComplexCommand::GoToClimb() {
  
  // 提前把底盘指针拿出来，避免在 Lambda 里使用 this
  auto drive = m_drivesubsystem;

  // 使用最清爽的 DeferredProxy
  return frc2::cmd::DeferredProxy([drive]() {
    double innerX = 3.2;
    double innerY = 5.4;
    double innerR = -45; 
    double OuterX = 6;
    double OuterY = 5.4;
    double OuterR = -45;

    auto alliance = frc::DriverStation::GetAlliance();
    
    // 全部使用局部的 drive 指针，断绝一切内存生命周期烦恼
    double x = drive->GetState().Pose.X().value();
    double y = drive->GetState().Pose.Y().value(); 

    double targetSpeed = 2.5;
    bool invertD = y <= 4;
    bool invertA = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
    
    // 根据实时 X 坐标决定先后顺序
    if (x > 5 && x < 11.54) {
      innerX-=0.5;
      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, OuterX, OuterY, OuterR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveClosed(drive, innerX, innerY, innerR, targetSpeed, invertA, invertD).ToPtr()
      ); 
    } else {
      OuterX+=0.5;
      return frc2::cmd::Sequence(
        AutoMoveOpen(drive, innerX, innerY, innerR, targetSpeed, invertA, invertD).ToPtr(),
        AutoMoveClosed(drive, OuterX, OuterY, OuterR, targetSpeed, invertA, invertD).ToPtr()
      ); 
    }
  });
}