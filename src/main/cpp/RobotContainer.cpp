// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
    : visionSub(&drivetrain, nullptr),
      shooterSub(joystick),
      feederSub(joystick),
      weidaiSub(joystick),
      groundIntakeSub(joystick),
      complexcommand(&drivetrain, &visionSub, &shooterSub, &feederSub, &weidaiSub,
                     &groundIntakeSub)
{
  shooterSub.SetFeederSubsystem(&feederSub);
  shooterSub.SetDrivetrainSubsystem(&drivetrain);
  drivetrain.setShooterSubSystem(&shooterSub);
  // 濞夈劌鍞絘uto娑撶挱vents閸涙垝鎶?
  // EventTrigger("Reset").OnTrue(ResetTranslationCommand());
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);
  rot.EnableContinuousInput(-180,180);

  m_autoChooser.SetDefaultOption("Do Nothing (Safe)", AutoMode::kDoNothing);
  // 2. 添加你手写的所有高阶路线
  m_autoChooser.AddOption("OutAndDepot", AutoMode::OutDepot);
  m_autoChooser.AddOption("DoubleAutoLeft", AutoMode::DoubleOutLeft);
  m_autoChooser.AddOption("DoubleAutoRight", AutoMode::DoubleOutRight);
  m_autoChooser.AddOption("OutAndHP", AutoMode::OutAndHP);
  // m_autoChooser.AddOption("OutAndDepotRed", AutoMode::RedOutDepot);
  // m_autoChooser.AddOption("DoubleOutLeftRed",AutoMode::RedDoubleOutLeft);
  // m_autoChooser.AddOption("DoubleOutRightRed",AutoMode::RedDoubleOutRight);
  // m_autoChooser.AddOption("OutAndHPRed",AutoMode::RedOutAndHP);
  m_autoChooser.AddOption("LowLeft",AutoMode::LowLeft);
  m_autoChooser.AddOption("LowRight", AutoMode::LowRight);
  m_autoChooser.AddOption("LowLeftNoBounce",AutoMode::LowLeftNoBounce);
  m_autoChooser.AddOption("LowRightNoBounce", AutoMode::LowRightNoBounce);
  // 3. 推送到 Shuffleboard / SmartDashboard
  frc::SmartDashboard::PutData("Auto Mode", &m_autoChooser);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Input ownership note:
  // Periodic-owned inputs: LeftTrigger, RightTrigger.
  // Check docs/controller-input-map.md before adding new button bindings.
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
 #include <algorithm> // 为了使用 std::clamp
#include <frc/DriverStation.h>

drivetrain.SetDefaultCommand(
    drivetrain.ApplyRequest([this]() ->auto&& {
        
        // 1. 获取基础手柄输入
        double rawVx = -joystick.GetLeftY(); // 对应底盘 X 轴 (前后)
        double rawVy = -joystick.GetLeftX(); // 对应底盘 Y 轴 (左右)
        
        // 算出平移速度指令 (不论什么模式都要用到)
        auto vXCmd = rawVx * MaxSpeed * OperatorConstants::SpeedRate;
        auto vYCmd = rawVy * MaxSpeed * OperatorConstants::SpeedRate;
        if(ground_intake_prepared_){
          vXCmd*=1;
          vYCmd*=1;
        }
        if(driveLikeTank) {
            double rate = 0.0; // 默认不旋转
            
            // 计算摇杆推力大小
            double magnitude = std::sqrt(rawVx*rawVx+rawVy*rawVy);

            if (magnitude > 0.05) {
                // 处理红蓝方反转
                bool isRed = frc::DriverStation::GetAlliance().has_value() && 
                             frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed;
                
                double fieldX = isRed ? -rawVx : rawVx;
                double fieldY = isRed ? -rawVy : rawVy;
                
                double targetDeg = std::atan2(fieldY, fieldX) / PI * 180.0;
                
                rate = rot.Calculate(drivetrain.GetState().Pose.Rotation().Degrees().value(), targetDeg);
                double maxRate = (MaxAngularRate * OperatorConstants::AngularSpeedRate/PI*180).value();
                rate = std::clamp(rate, -maxRate, maxRate);
            }
            
            return driveClosed
                .WithVelocityX(vXCmd)
                .WithVelocityY(vYCmd)
                .WithRotationalRate(units::degrees_per_second_t{rate});
        }
        else {
            return driveClosed
                .WithVelocityX(vXCmd)
                .WithVelocityY(vYCmd)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate * OperatorConstants::AngularSpeedRate);
        }
    })
);

  // joystick.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] {
  //   useClosedLoop = !useClosedLoop;
  //   frc::SmartDashboard::PutBoolean("Drive ClosedLoop", useClosedLoop);
  // }));
  
  joystick.Start().OnTrue(
    frc2::cmd::Sequence(
    complexcommand.StartStorageCommand(),
    frc2::cmd::WaitUntil([this] { return weidaiSub.GetStorageNormPosition() > 0.7; }),
    complexcommand.GroundintakeresetCommand(),
    groundIntakeSub.SetPitchNormPositionCommandPtr(0.20),
    frc2::cmd::WaitUntil([this] { return groundIntakeSub.GetPitchNormPosition() <= 0.08; })
        .WithTimeout(units::second_t{1.0}),
    complexcommand.CloseStorageCommand(),
    frc2::cmd::RunOnce([this]{ground_intake_prepared_=false;})
  ));

  joystick.RightTrigger()
  .OnTrue(complexcommand.GroundintakeresetCommand())
  .WhileTrue(
    frc2::cmd::Either(
        frc2::cmd::Parallel(
            // 1. 底盘负责瞄准
            RealTimeAimDrive(
                &drivetrain, &shooterSub,
                [this]() { return -joystick.GetLeftY(); }, 
                [this]() { return -joystick.GetLeftX(); },180  
            ).ToPtr(),
            
            // 2. 机制负责射球
            frc2::cmd::Sequence(
                shooterSub.EnableShooter(),
                complexcommand.GroundintakeresetCommand(),
                frc2::cmd::RunOnce([this] { ground_intake_prepared_ = false;}),
                frc2::cmd::RunOnce([this] {shooterEnabled = true; }),
                frc2::cmd::Parallel(
                complexcommand.ShootWithFeederCommand(),
                frc2::cmd::Sequence(
                frc2::cmd::WaitUntil([this]{return 
            std::abs(shooterSub.GetShootVelocity()-shooterSub.realShootVelocity)<0.7
            &&drivetrain.SOMangleDiff<=8;})
          .WithTimeout(units::second_t{1.5}),
                complexcommand.GroundintakeassistCommand().Repeatedly()))
            )
        ),

        frc2::cmd::Parallel(
            // 1. 底盘和 Feeder 已经被 PassBallCommand 包办了
            PassBallCommand(
                &drivetrain, &shooterSub, &feederSub,
                [this]() { return -joystick.GetLeftY(); }, 
                [this]() { return -joystick.GetLeftX(); },180
            ).ToPtr(),
            // 2. 传球时附带的额外动作
            frc2::cmd::Sequence(
                complexcommand.GroundintakeresetCommand(),
                shooterSub.EnableShooter(),
                frc2::cmd::RunOnce([this] {  shooterEnabled = true; }),
                frc2::cmd::WaitUntil([this]{return 
                  std::abs(shooterSub.GetShootVelocity()-shooterSub.realShootVelocity)<0.7
                  &&drivetrain.SOMangleDiff<=8;}).WithTimeout(units::second_t{1.5}),
                frc2::cmd::Sequence(
                    frc2::cmd::Sequence(
                      complexcommand.GroundintakeassistCommand(),
                      frc2::cmd::RunOnce([this] { ground_intake_prepared_ = false;}))
                    .OnlyIf([this]{
                      frc::Translation2d nowT=drivetrain.GetState().Pose.Translation();
                      return nowT.Y().value()<=3.5||nowT.Y().value()>=4.5;})
                ).Repeatedly()
            )
        ),
        [this] {
            auto alliance = frc::DriverStation::GetAlliance();
            bool isRed=alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed;
            double x = drivetrain.GetState().Pose.X().value();
            return (!isRed&&x<=5.0) || (isRed&&x>=11.54);
        }
    )
).OnFalse(
    frc2::cmd::Sequence(
        complexcommand.StopShootWithFeederCommand(),
        //complexcommand.GroundintakeresetCommand(),
        frc2::cmd::RunOnce([this] { shooterEnabled = false; })
    )
);

  joystick.POVLeft().OnTrue(frc2::cmd::Either(
    shooterSub.EnableShooter(),
    complexcommand.StopShootWithFeederCommand(),
    [this]{return !shooterEnabled;}
  ).AndThen(frc2::cmd::RunOnce(
              [this] { shooterEnabled = !shooterEnabled; }))
  );

// 根据storage位置切换
joystick.LeftBumper().OnTrue(
    frc2::cmd::Either(
        complexcommand.StartStorageCommand(),
        complexcommand.CloseStorageCommand(),
        [this] { return weidaiSub.GetStorageNormPosition() < 0.5; }
    )
);
 
  // joystick.B().WhileTrue(
  //     frc2::cmd::StartEnd(銆併€戙€愩€戙€愩€?
  //         [this]
  //         { shooterSub.SetShootVelocity(44
  //         ); }, // 瀵偓婵妞傞幍褑顢?
  //         [this]
  //         { shooterSub.Stop(); }, //
  //         缂佹挻娼敍鍫熸緱瀵偓閿涘妞傞幍褑顢?
  //         {&shooterSub}           // 閸忔娊鏁敍姘紣閺勫酣娓跺Ч?
  //         ));

  // //鎼存洜娲?SysId閵嗘垯鈧降鈧?
  // joystick.A().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // // Shooter SysId
  // joystick.A().WhileTrue(shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(shooterSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(shooterSub.SysIdDynamic(frc2::sysid::Direction::kReverse));
  // // Feeder SysId
  // joystick.A().WhileTrue(feederSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // joystick.B().WhileTrue(feederSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // joystick.X().WhileTrue(feederSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // joystick.Y().WhileTrue(feederSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  drivetrain.RegisterTelemetry(
       [this](auto const& state) { logger.Telemeterize(state); });

  joystick.RightBumper().WhileTrue(
    frc2::cmd::Sequence(
      //complexcommand.StopShootWithFeederCommand(),
      complexcommand.GroundintakeresetCommand(),
      frc2::cmd::Either(
        complexcommand.PassBump(false),
        complexcommand.PassBump(true),
      [this]{
        auto alliance=frc::DriverStation::GetAlliance();
        double nowX=drivetrain.GetState().Pose.X().value();
        return (alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kBlue&&nowX<=16.54/2)||(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed&&nowX>=16.54/2);
      })
  ));

  //增加使用默认速度选项，以防Limelight出问题
  joystick.POVUp().OnTrue(shooterSub.EnableDefaultShoot());
  joystick.POVDown().OnTrue(shooterSub.DisableDefaultShoot());
  joystick.POVRight().OnTrue(complexcommand.GroundintakeantiCommand()).OnFalse(complexcommand.GroundintakeresetCommand());

  joystick.A().WhileTrue(
    frc2::cmd::Parallel(
      complexcommand.StartStorageCommand(),
    intakeNextToSide(          
            &drivetrain,  
            &shooterSub, 
            &groundIntakeSub,
            &joystick,
            true ).ToPtr())).OnFalse(frc2::cmd::RunOnce( [this] { ground_intake_prepared_ = false;}));

  joystick.LeftTrigger().OnFalse(

        frc2::cmd::Sequence(
          complexcommand.GroundintakeresetCommand(),
          frc2::cmd::RunOnce( [this] { ground_intake_prepared_ = false;})))
          .WhileTrue(
            frc2::cmd::Sequence(
              frc2::cmd::RunOnce( [this] { ground_intake_prepared_ = true;}),
            frc2::cmd::Parallel(
              complexcommand.GroundintakeprepareCommand(),
              complexcommand.StartStorageCommand().Repeatedly())
            ));


   joystick.Y()
   .WhileTrue(
    frc2::cmd::Sequence(
      //complexcommand.StopShootWithFeederCommand(),
      complexcommand.CloseStorageCommand(),
      //complexcommand.GroundintakeresetCommand(),
      frc2::cmd::Either(
        complexcommand.PassTrench(false),
        complexcommand.PassTrench(true),
      [this]{
        auto alliance=frc::DriverStation::GetAlliance();
        double nowX=drivetrain.GetState().Pose.X().value();
        return (alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kBlue&&nowX<=16.54/2)||(alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed&&nowX>=16.54/2);
      })
  ));
  // joystick.LeftBumper().OnTrue(
  //     complexcommand.GroundintakeassistCommand()
  //         .AndThen(frc2::cmd::RunOnce(
  //             [this] { ground_intake_prepared_ = false; })));
  

  
  joystick.B().WhileTrue(
    frc2::cmd::Parallel(
      complexcommand.StartStorageCommand(),
   frc2::cmd::Either(
        intakeNextToHub(
            &drivetrain,     // 你的 CommandSwerveDrivetrain 实例指针
            &shooterSub,   // 你的 ShooterSubsystem 实例指针
            &groundIntakeSub,
            &joystick,
            true    // 你的 GroundIntakeSubsystem 实例指针
        ).ToPtr(), 
        intakeNextToWall(&drivetrain, &shooterSub, &groundIntakeSub,&joystick, true).ToPtr(),
        [this]{
          double nowX=drivetrain.GetState().Pose.X().value();
          return nowX>5&&nowX<11.54;}
              ))).OnFalse(frc2::cmd::RunOnce( [this] { ground_intake_prepared_ = false;}));    // 转换为 WPILib 最新架构的 CommandPtr
  
  
  joystick.X().WhileTrue(
    frc2::cmd::Parallel(
      complexcommand.StartStorageCommand(),
    frc2::cmd::Either(
        intakeNextToHub(
            &drivetrain,     // 你的 CommandSwerveDrivetrain 实例指针
            &shooterSub,   // 你的 ShooterSubsystem 实例指针
            &groundIntakeSub,
            &joystick,
            false    // 你的 GroundIntakeSubsystem 实例指针
        ).ToPtr(), 
        intakeNextToWall(&drivetrain, &shooterSub, &groundIntakeSub, &joystick,false).ToPtr(),
        [this]{
          double nowX=drivetrain.GetState().Pose.X().value();
          return nowX>5&&nowX<11.54;}
              ))).OnFalse(frc2::cmd::RunOnce( [this] { ground_intake_prepared_ = false;}));

}

frc2::CommandPtr RobotContainer::GenerateAutoCommand() {
    AutoMode selectedMode = m_autoChooser.GetSelected();
    auto alliance=frc::DriverStation::GetAlliance();
    bool isRed=alliance.has_value()&&alliance.value()==frc::DriverStation::Alliance::kRed;

  switch (selectedMode) {
    case AutoMode::OutDepot:
      return autos::AutoSlowLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed, isRed);
      
    case AutoMode::DoubleOutLeft:
      return autos::AutoDoubleLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed, isRed);
      
    case AutoMode::DoubleOutRight:
      return autos::AutoDoubleLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand,isRed, !isRed);
      
    case AutoMode::OutAndHP:
      return autos::AutoHP(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand,isRed, isRed);
    
    case AutoMode::RedDoubleOutLeft:
      return autos::AutoDoubleLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, true,true);

    case AutoMode::RedDoubleOutRight:
      return autos::AutoDoubleLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, true,false);

    case AutoMode::RedOutAndHP:
      return autos::AutoHP(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand,true, true);

    case AutoMode::RedOutDepot:
      return autos::AutoSlowLeft(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, true, true);
    
    case AutoMode::LowLeft:
      return autos::AutoLow(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed,isRed);
    
    case AutoMode::LowRight:
      return autos::AutoLow(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed,!isRed);
    case AutoMode::LowRightNoBounce:
      return autos::AutoLowOnlyOneSide(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed,!isRed);
    case AutoMode::LowLeftNoBounce:
      return autos::AutoLowOnlyOneSide(&drivetrain, &shooterSub, &feederSub, &groundIntakeSub, &complexcommand, isRed,isRed);
    
    case AutoMode::kDoNothing:
    default:
      return frc2::cmd::None();
  }
}

void RobotContainer::refreshAutoMode(){
  // 1. 获取当前的 Enum 选择
    AutoMode currentSelection = m_autoChooser.GetSelected();

    // 2. 如果发生了切换，或者缓存是空的
    if (currentSelection != m_lastSelectedAuto || !m_preloadedAuto.has_value()) {

        frc::SmartDashboard::PutString("Auto Status", "WAITING!");
        m_lastSelectedAuto = currentSelection;
        // 3. 重新生成并装入盒子
        m_preloadedAuto = RobotContainer::GenerateAutoCommand();
        // 因为 enum 不能直接打印成字符串，我们可以简单地在面板上发个 Ready 信号
        frc::SmartDashboard::PutString("Auto Status", "READY!");
    }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // 1. 检查盒子里有没有提前做好的指令
    if (m_preloadedAuto.has_value()) {
        // 2. 极其关键的 std::move！
        // 把控制权从 optional 转移给 WPILib 调度器
        frc2::CommandPtr autoCmd = std::move(m_preloadedAuto.value());
        // 3. 清空盒子。这样下次再回到 Disabled 状态时，它会重新生成
        m_preloadedAuto.reset(); 
        return autoCmd; // 瞬间返回，底层直接起飞！
    }
    // 万一出错了（比如刚开机还没过 Disabled 就强行进 Auto），给个保底的空指令
    return frc2::cmd::None();
}
