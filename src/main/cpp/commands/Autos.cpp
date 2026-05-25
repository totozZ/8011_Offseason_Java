// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

frc2::CommandPtr autos::AutoSlowLeft(CommandSwerveDrivetrain* drivetrain, ShooterSubsystem* shooter, 
                                    FeederSubsystem* feeder, GroundIntakeSubsystem* intaker, ComplexCommand* complexcommand, bool invertA, bool invertD) {
    
    // 1. 获取 Index 0 的初始点
    double x = slowXLeft[0];
    double y = slowYLeft[0];
    double d = slowRLeft[0];

    // 2. 根据翻转布尔值处理初始点坐标
    if(invertD){
        d = 0.0 - d;
        y = 8.07 - y;
    }
    if(invertA){
        x = 16.54 - x;
        d = 180.0 - d;
    }

    // abcdef是drivetrain里的一个全局swerve request变量
    drivetrain->abcdef.WithHeadingPID(8, 0, 0.1)
                    .WithDeadband(units::meters_per_second_t{4}* 0.05)
                    .WithRotationalDeadband(units::radians_per_second_t{0.1})
            .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14})
            .WithDriveRequestType(swerve::DriveRequestType::Velocity)
            .WithSteerRequestType(swerve::SteerRequestType::Position);
    
    frc::Rotation2d sR=drivetrain->GetState().Pose.Rotation();

    // 包装初始位姿
    frc::Pose2d start{units::meter_t{x}, units::meter_t{y}, sR};
    return frc2::cmd::Sequence(
        // frc2::cmd::RunOnce([drivetrain, start] {
        //     drivetrain->ResetPose(start); 
        // }, {drivetrain}),
        AutoMoveOpen(drivetrain, 7.0, slowYLeft[1], slowRLeft[1], slowSLeft[1], invertA, invertD).ToPtr().Until([invertA,intaker,drivetrain]{
          double currentX=drivetrain->GetState().Pose.X().value();
          return intaker->getPitchResetFlag()&&((currentX>slowXLeft[1]&&!invertA)||(currentX<16.54-slowXLeft[1]&&invertA));}),
        complexcommand->GroundintakeprepareCommand(),
        
        // AutoMoveOpen(drivetrain, slowXLeft[2], slowYLeft[2], slowRLeft[2], slowSLeft[2], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, slowXLeft[3], slowYLeft[3], slowRLeft[3], slowSLeft[3], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, slowXLeft[3]-0.8, slowYLeft[3]+0.1,0.4,30,false, slowSLeft[3],false,-90,false, invertA, invertD,0).ToPtr(),
        AutoMoveOpen(drivetrain, slowXLeft[4], slowYLeft[4], slowRLeft[4], slowSLeft[4], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, slowXLeft[4]-0.65,slowYLeft[4]+0.2,0.65,-150,false, 2.4,false,180,true,invertA, invertD,0).ToPtr(),
        
        AutoMoveOpen(drivetrain, slowXLeft[5], slowYLeft[5], slowRLeft[5], slowSLeft[5], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        AutoMoveClosed(drivetrain, slowXLeft[6], slowYLeft[6], slowRLeft[6], slowSLeft[6], invertA, invertD).ToPtr(),

        frc2::cmd::Parallel(
        RealTimeAimDrive(drivetrain,shooter, []{return 0;},[]{return 0;},180).ToPtr(),
        frc2::cmd::Parallel(
          complexcommand->ShootWithFeederCommand(),
          complexcommand->GroundintakeassistCommand().Repeatedly()
        )
        ).WithTimeout(units::second_t{2.5}),
        shooter->DisableShooter(),
        AutoMoveClosed(drivetrain, slowXLeft[8], slowYLeft[8], slowRLeft[8], slowSLeft[8], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeprepareCommand(),        
        AutoMoveOpen(drivetrain, slowXLeft[9], slowYLeft[9], slowRLeft[9], slowSLeft[9], invertA, invertD).ToPtr(),
        AutoMoveClosed(drivetrain, slowXLeft[10], slowYLeft[10], slowRLeft[10], slowSLeft[10], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        AutoMoveClosed(drivetrain, slowXLeft[11], slowYLeft[11], slowRLeft[11], slowSLeft[11], invertA, invertD).ToPtr(),
        shooter->EnableShooter(), // 提前预热飞轮
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2}),
        complexcommand->StopShootWithFeederCommand(),
        // AutoMoveOpen(drivetrain, slowXLeft[12], slowYLeft[12], slowRLeft[12], slowSLeft[12], invertA, invertD).ToPtr(),
        complexcommand->CloseStorageCommand(),
        complexcommand->PassTrench(false).Until([drivetrain, invertA]{return (!invertA&&drivetrain->GetState().Pose.X().value()>AutoXLow[8]+1)||(invertA&&drivetrain->GetState().Pose.X().value()<16.54-AutoXLow[8]-1);}),
        intakeNextToHub(
            drivetrain,     // 你的 CommandSwerveDrivetrain 实例指针
            shooter,   // 你的 ShooterSubsystem 实例指针
            intaker,
            nullptr,
            true    // 你的 GroundIntakeSubsystem 实例指针
        ).ToPtr()
    );
}

frc2::CommandPtr autos::AutoDoubleLeft(CommandSwerveDrivetrain* drivetrain, ShooterSubsystem* shooter, 
                                     FeederSubsystem* feeder, GroundIntakeSubsystem* intaker, ComplexCommand* complexcommand, bool invertA, bool invertD) {
    
    // 1. 获取 Index 0 的初始点
    double x = AutoXDouble[0];
    double y = AutoYDouble[0];
    double d = AutoRDouble[0];
       
    // 2. 根据翻转布尔值处理初始点坐标
    if(invertD){
        d = 0.0 - d;
        y = 8.07 - y;
    }
    if(invertA){
        x = 16.54 - x;
        d = 180.0 - d;
    }
    frc::Rotation2d sR=drivetrain->GetState().Pose.Rotation();

    // 包装初始位姿
    frc::Pose2d start{units::meter_t{x}, units::meter_t{y}, sR};

    return frc2::cmd::Sequence(
        // frc2::cmd::RunOnce([drivetrain, start] {
        //     drivetrain->ResetPose(start); 
        // }, {drivetrain}),
        complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXDouble[1], AutoYDouble[1], AutoRDouble[1], AutoSDouble[1], invertA, invertD).ToPtr(),
        complexcommand->StartStorageCommand(),
        // AutoMoveOpen(drivetrain, AutoXDouble[2], AutoYDouble[2], AutoRDouble[2], AutoSDouble[2], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, AutoXDouble[3], AutoYDouble[3], AutoRDouble[3], AutoSDouble[3], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXDouble[3]-0.6, AutoYDouble[3]-0.5, 0.1,30,false,AutoSDouble[3],false,-70,false,invertA,invertD,0 ).ToPtr(),

        AutoMoveOpen(drivetrain, AutoXDouble[4], AutoYDouble[4]+0.2, AutoRDouble[4], AutoSDouble[4], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXDouble[4]-0.65,AutoYDouble[4]+0.2,0.65,-150,false, AutoSDouble[4],false,180,true,invertA, invertD,0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXDouble[5], AutoYDouble[5], AutoRDouble[5], AutoSDouble[5], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        AutoMoveClosed(drivetrain, AutoXDouble[6], AutoYDouble[6], AutoRDouble[6], AutoSDouble[6], invertA, invertD).ToPtr(),

        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.5}),
        complexcommand->StopShootWithFeederCommand(),
        complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXDouble[7], AutoYDouble[7], AutoRDouble[7], AutoSDouble[7], invertA, invertD).ToPtr(),

        AutoMoveOpen(drivetrain, AutoXDouble[8], AutoYDouble[8], AutoRDouble[8], AutoSDouble[8], invertA, invertD).ToPtr(),
        
        // AutoMoveOpen(drivetrain, AutoXDouble[9], AutoYDouble[9], AutoRDouble[9], AutoSDouble[9], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, AutoXDouble[10], AutoYDouble[10], AutoRDouble[10], AutoSDouble[10], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, AutoXDouble[11], AutoYDouble[11], AutoRDouble[11], AutoSDouble[11], invertA, invertD).ToPtr(),

        AutoMoveCircle(drivetrain,(AutoXDouble[8]+AutoXDouble[11])/2,(AutoYDouble[8]+AutoYDouble[11])/2
        ,std::abs(AutoXDouble[8]-AutoXDouble[11])/2,0,true,AutoSDouble[9],false,0,true,invertA,invertD,0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXDouble[12], AutoYDouble[12], AutoRDouble[12], AutoSDouble[12], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, AutoXDouble[13], AutoYDouble[13], AutoRDouble[13], AutoSDouble[13], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXDouble[12]-0.8, AutoYDouble[12], 0.8,90,true,AutoSDouble[3],false,-90,true,invertA,invertD,0 ).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        AutoMoveOpen(drivetrain, AutoXDouble[14], AutoYDouble[14], AutoRDouble[14], AutoSDouble[14], invertA, invertD).ToPtr(),
        shooter->EnableShooter(),
        AutoMoveClosed(drivetrain, AutoXDouble[15], AutoYDouble[15], AutoRDouble[15], AutoSDouble[15], invertA, invertD).ToPtr(),
        
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.5}),
        complexcommand->StopShootWithFeederCommand(),
        AutoMoveOpen(drivetrain, slowXLeft[12], slowYLeft[12], slowRLeft[12], slowSLeft[12], invertA, invertD).ToPtr(),
        intakeNextToHub(drivetrain, shooter,intaker,nullptr,true).ToPtr()


    );
}

frc2::CommandPtr autos::AutoHP(CommandSwerveDrivetrain* drivetrain, ShooterSubsystem* shooter, 
                               FeederSubsystem* feeder, GroundIntakeSubsystem* intaker, ComplexCommand* complexcommand, bool invertA, bool invertD) {
    
    // 1. 获取 Index 0 的初始点
    double x = AutoXHP[0];
    double y = AutoYHP[0];
    double d = AutoRHP[0];
       
    // 2. 根据翻转布尔值处理初始点坐标
    if(invertD){
        d = 0.0 - d;
        y = 8.07 - y;
    }
    if(invertA){
        x = 16.54 - x;
        d = 180.0 - d;
    }
    frc::Rotation2d sR=drivetrain->GetState().Pose.Rotation();
    // 包装初始位姿
    frc::Pose2d start{units::meter_t{x}, units::meter_t{y}, sR};
    return frc2::cmd::Sequence(
        complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXHP[1], AutoYHP[1], AutoRHP[1], AutoSHP[1], invertA, invertD).ToPtr(),
             
        // AutoMoveOpen(drivetrain, AutoXHP[2], AutoYHP[2], AutoRHP[2], AutoSHP[2], invertA, invertD).ToPtr(),
        // AutoMoveOpen(drivetrain, AutoXHP[3], AutoYHP[3], AutoRHP[3], AutoSHP[3], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXHP[3]-0.8, AutoYHP[3]+0.1, 0.8,-30,true,AutoSHP[3],false,90,false,invertA,invertD,0 ).ToPtr(),
        AutoMoveOpen(drivetrain, AutoXHP[4], AutoYHP[4], AutoRHP[4], AutoSHP[4], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXHP[4]-0.65,AutoYHP[4]-0.2,0.65,150,true, AutoSHP[4],false,180,true,invertA, invertD,0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXHP[5], AutoYHP[5], AutoRHP[5], AutoSHP[5], invertA, invertD).ToPtr(),
        
        complexcommand->GroundintakeresetCommand(),   // 收回地吸
        shooter->EnableShooter(),                     // 提前预热飞轮
        
        AutoMoveClosed(drivetrain, AutoXHP[6], AutoYHP[6], AutoRHP[6], AutoSHP[6], invertA, invertD).ToPtr(),

      
       frc2::cmd::Parallel(
        RealTimeAimDrive(drivetrain, shooter,[]{return 0;},[]{return 0;},180).ToPtr(),
        frc2::cmd::Parallel(
          complexcommand->ShootWithFeederCommand(),
            complexcommand->GroundintakeassistCommand().Repeatedly())
        ).WithTimeout(units::second_t{3}),
        shooter->DisableShooter(),
        AutoMoveClosed(drivetrain, AutoXHP[8], AutoYHP[8], AutoRHP[8], AutoSHP[8], invertA, invertD).ToPtr(),
        frc2::cmd::Wait(units::second_t{2.0}),
        shooter->EnableShooter(), // 再次预热飞轮
        // 到达 Index 9 射击
        AutoMoveOpen(drivetrain, AutoXHP[9], AutoYHP[9], AutoRHP[9], AutoSHP[9], invertA, invertD).ToPtr().WithTimeout(units::second_t{2}),
        
        // 边自瞄边吸球辅助 - Index 10
        AutoMoveClosed(drivetrain, AutoXHP[10], AutoYHP[10], AutoRHP[10], AutoSHP[10], invertA, invertD).ToPtr(),
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).Until([]{return frc::Timer::GetMatchTime()<=units::second_t{1.5};}),
        complexcommand->StopShootWithFeederCommand(),

        
        // 最后的跑点/射击 - Index 11
        AutoMoveClosed(drivetrain, AutoXHP[11], AutoYHP[11], AutoRHP[11], AutoSHP[11], invertA, invertD).ToPtr().WithTimeout(units::second_t{2})
    );
}

frc2::CommandPtr autos::AutoLow(CommandSwerveDrivetrain* drivetrain, ShooterSubsystem* shooter, 
FeederSubsystem* feeder, GroundIntakeSubsystem* intaker, ComplexCommand* complexcommand, bool invertA, bool invertD){
      return frc2::cmd::Sequence(
        complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXLow[1], AutoYLow[1], AutoRLow[1], AutoSLow[1], invertA, invertD).ToPtr(),
        // frc2::cmd::Parallel(
        //   AutoMoveOpen(drivetrain, AutoXLow[1], AutoYLow[1], AutoRLow[1], AutoSLow[1], invertA, invertD).ToPtr(),
        //  frc2::cmd::Sequence(
        //   frc2::cmd::WaitUntil([intaker]{return intaker->getPitchResetFlag();}),
        //   complexcommand->GroundintakeprepareCommand().Repeatedly()
          
        //  )).Until([drivetrain,invertA]{double currentX=drivetrain->GetState().Pose.X().value(); return (currentX>AutoXLow[1]&&!invertA)||(currentX<16.54-AutoXLow[1]&&invertA);}),
        
      
        // AutoMoveOpen(drivetrain, 7, AutoYLow[1], AutoRLow[1], AutoSLow[1], invertA, invertD).ToPtr().Until([invertA,intaker,drivetrain]{
        //   double currentX=drivetrain->GetState().Pose.X().value();
        //   return intaker->getPitchResetFlag()&&((currentX>AutoXLow[1]&&!invertA)||(currentX<16.54-AutoXLow[1]&&invertA));}),
        // complexcommand->StartStorageCommand(),
        //frc2::cmd::Parallel(
        complexcommand->StartStorageCommand(),//.Repeatedly().Until([intaker]{return intaker->GetPitchNormPosition()>0.8;}),
        frc2::cmd::Race(
        AutoMoveCircle(drivetrain, AutoXLow[2]-0.8, AutoYLow[2]+0.1,0.4,30,false, AutoSLow[2],false,-90,false, invertA, invertD,0).ToPtr(),
       // ),
        frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([intaker]{return intaker->getPitchResetFlag();}),
          complexcommand->GroundintakeprepareCommand().Repeatedly()
          
         )
       ),
        AutoMoveOpen(drivetrain, AutoXLow[3], AutoYLow[3]+0.3, AutoRLow[3], AutoSLow[3], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[3]-0.8,AutoYLow[3]+0.3,0.8,-130,false, AutoSLow[1]-0.4,false,0,true,invertA, invertD,0).ToPtr(),
        AutoMoveOpen(drivetrain, AutoXLow[4], AutoYLow[4], AutoRLow[4], AutoSLow[4], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        AutoMoveClosed(drivetrain, AutoXLow[5], AutoYLow[5], AutoRLow[5], AutoSLow[5], invertA, invertD).ToPtr(),
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
            complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.8}),
        complexcommand->StopShootWithFeederCommand(),
        shooter->DisableShooter(),
        //intaker->SetPitchNormPositionCommandPtr(0.965),
        AutoMoveOpen(drivetrain, AutoXLow[6], AutoYLow[6]-0.3, AutoRLow[6], AutoSLow[6], invertA, invertD).ToPtr().Until([drivetrain, invertA]{
          double r = drivetrain->GetState().Pose.Rotation().Degrees().value();
          return (!invertA&&r<80&&r>-80)||(invertA&&(r>100||r<-100));}),
        //AutoMoveCircle(drivetrain, AutoXLow[7]-0.2, AutoYLow[6],std::abs(AutoYLow[7]-AutoYLow[6]), 90,false, AutoSLow[7],false,AutoRLow[7],false,invertA, invertD,0).ToPtr(),
        complexcommand->CloseStorageCommand(),
        complexcommand->GroundintakeprepareCommand(),
        complexcommand->PassTrench(false).Until([drivetrain, invertA]{return (!invertA&&drivetrain->GetState().Pose.X().value()>AutoXLow[8])||(invertA&&drivetrain->GetState().Pose.X().value()<16.54-AutoXLow[8]);}),
        complexcommand->StartStorageCommand(),
        //AutoMoveOpen(drivetrain, AutoXLow[8], AutoYLow[8], AutoRLow[8], AutoSLow[8], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[9]-0.9, AutoYLow[9],0.7,0,false, AutoSLow[9],false,-90,false, invertA, invertD,0).ToPtr(),
        AutoMoveOpen(drivetrain, AutoXLow[10], AutoYLow[10], AutoRLow[10], AutoSLow[10], invertA, invertD).ToPtr(),
        
        AutoMoveCircle(drivetrain, (AutoXLow[10] + AutoXLow[11]) / 2, (AutoYLow[10] + AutoYLow[11]) / 2, 
                       std::abs(AutoXLow[10] - AutoXLow[11]) / 2, 0, true, AutoSLow[11], false, 0, true, invertA, invertD,0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXLow[12], AutoYLow[12], AutoRLow[12], AutoSLow[12], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[12] - 0.8, AutoYLow[12], 0.8, 90, true, AutoSLow[12], false, -90, true, invertA, invertD, 0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXLow[14], AutoYLow[14], AutoRLow[14], AutoSLow[14], invertA, invertD).ToPtr(),
        // AutoMoveCircle(drivetrain, AutoXLow[14] - 0.5, AutoYLow[14]+0.5, 0.1, -90, false, AutoSLow[14],false, -90, true, invertA, invertD, 0).ToPtr(),
        
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        
        AutoMoveClosed(drivetrain, AutoXLow[15], AutoYLow[15], AutoRLow[15], AutoSLow[15], invertA, invertD).ToPtr(),
        
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
              complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.5}),
        complexcommand->StopShootWithFeederCommand(),
        // AutoMoveOpen(drivetrain, slowXLeft[12], slowYLeft[12], slowRLeft[12], slowSLeft[12], invertA, invertD).ToPtr(),
        // intakeNextToHub(drivetrain,shooter,intaker,true).ToPtr()
        complexcommand->CloseStorageCommand(),
        complexcommand->PassTrench(false).Until([drivetrain, invertA]{return (!invertA&&drivetrain->GetState().Pose.X().value()>AutoXLow[8])||(invertA&&drivetrain->GetState().Pose.X().value()<16.54-AutoXLow[8]);}),
        AutoMoveOpen(drivetrain, 11, 4, 180, 2.70, invertA, invertD).ToPtr()
       

      );

    }
frc2::CommandPtr autos::AutoLowOnlyOneSide(CommandSwerveDrivetrain* drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intaker, ComplexCommand* complexcommand, bool invertA, bool invertD){
      return frc2::cmd::Sequence(
        // AutoMoveOpen(drivetrain, 7, AutoYLow[1], AutoRLow[1], AutoSLow[1], invertA, invertD).ToPtr().Until([invertA,intaker,drivetrain]{
        //   double currentX=drivetrain->GetState().Pose.X().value();
        //   return intaker->getPitchResetFlag()&&((currentX>AutoXLow[1]&&!invertA)||(currentX<16.54-AutoXLow[1]&&invertA));}),
       complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXLow[1], AutoYLow[1], AutoRLow[1], AutoSLow[1], invertA, invertD).ToPtr(),
        complexcommand->StartStorageCommand(),
         complexcommand->GroundintakeprepareCommand(),
        frc2::cmd::Race(
        AutoMoveCircle(drivetrain, AutoXLow[2]-0.8, AutoYLow[2]+0.1,0.4,30,false, AutoSLow[2],false,-90,false, invertA, invertD,0).ToPtr(),
       // ),
        frc2::cmd::Sequence(
          frc2::cmd::WaitUntil([intaker]{return intaker->getPitchResetFlag();}),
          complexcommand->GroundintakeprepareCommand().Repeatedly()
          
         )
       ),
        AutoMoveOpen(drivetrain, AutoXLow[3], AutoYLow[3]+0.8, AutoRLow[3], AutoSLow[3], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[3]-0.8,AutoYLow[3]+0.8,0.8,-130,false, AutoSLow[1]-0.8,false,0,true,invertA, invertD,0).ToPtr(),
        AutoMoveOpen(drivetrain, AutoXLow[4], AutoYLow[4], AutoRLow[4], AutoSLow[4], invertA, invertD).ToPtr(),
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        AutoMoveClosed(drivetrain, AutoXLow[5], AutoYLow[5], AutoRLow[5], AutoSLow[5], invertA, invertD).ToPtr(),
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
            complexcommand->ShootWithFeederCommand(),
            complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.8}),
        complexcommand->StopShootWithFeederCommand(),
        shooter->DisableShooter(),
        //intaker->SetPitchNormPositionCommandPtr(0.965),
        AutoMoveOpen(drivetrain, AutoXLow[6], AutoYLow[6]-0.3, AutoRLow[6], AutoSLow[6], invertA, invertD).ToPtr().Until([drivetrain, invertA]{
          double r = drivetrain->GetState().Pose.Rotation().Degrees().value();
          return (!invertA&&r<80&&r>-80)||(invertA&&(r>100||r<-100));}),
        //AutoMoveCircle(drivetrain, AutoXLow[7]-0.2, AutoYLow[6],std::abs(AutoYLow[7]-AutoYLow[6]), 90,false, AutoSLow[7],false,AutoRLow[7],false,invertA, invertD,0).ToPtr(),
        complexcommand->CloseStorageCommand(),
        complexcommand->GroundintakeprepareCommand(),
        complexcommand->PassTrench(false).Until([drivetrain, invertA]{return (!invertA&&drivetrain->GetState().Pose.X().value()>AutoXLow[8])||(invertA&&drivetrain->GetState().Pose.X().value()<16.54-AutoXLow[8]);}),
        complexcommand->StartStorageCommand(),
        //AutoMoveOpen(drivetrain, AutoXLow[8], AutoYLow[8], AutoRLow[8], AutoSLow[8], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[9]-0.9, AutoYLow[9],0.7,0,false, AutoSLow[9],false,-90,false, invertA, invertD,0).ToPtr(),
        AutoMoveOpen(drivetrain, AutoXLow[10], AutoYLow[10]+1, AutoRLow[10], AutoSLow[10], invertA, invertD).ToPtr(),
        
        AutoMoveCircle(drivetrain, (AutoXLow[10] + AutoXLow[11]) / 2, (AutoYLow[10] + AutoYLow[11]) / 2+1.1, 
                       std::abs(AutoXLow[10] - AutoXLow[11]) / 2, 0, true, AutoSLow[11], false, 0, true, invertA, invertD,0).ToPtr(),

        AutoMoveOpen(drivetrain, AutoXLow[12], AutoYLow[12]+1.2, AutoRLow[12], AutoSLow[12], invertA, invertD).ToPtr(),
        AutoMoveCircle(drivetrain, AutoXLow[12] - 0.7, AutoYLow[12]+1.0, 0.7, 130, true, AutoSLow[12], false, -90, true, invertA, invertD, 0).ToPtr(),
        
        AutoMoveOpen(drivetrain, AutoXLow[14]-0.4, AutoYLow[14], 135, AutoSLow[14], invertA, invertD,2.0).ToPtr(),
        // AutoMoveCircle(drivetrain, AutoXLow[14] - 0.5, AutoYLow[14]+0.5, 0.1, -90, false, AutoSLow[14],false, -90, true, invertA, invertD, 0).ToPtr(),
        
        complexcommand->GroundintakeresetCommand(),
        shooter->EnableShooter(),
        
        AutoMoveClosed(drivetrain, AutoXLow[15], AutoYLow[15], AutoRLow[15], AutoSLow[15], invertA, invertD).ToPtr(),
        
        frc2::cmd::Parallel(
          RealTimeAimDrive(drivetrain, shooter, []{return 0;}, []{return 0;},180).ToPtr(),
            frc2::cmd::Parallel(
              complexcommand->ShootWithFeederCommand(),
              complexcommand->GroundintakeassistCommand().Repeatedly()
            )
        ).WithTimeout(units::second_t{2.5}),
        complexcommand->StopShootWithFeederCommand(),
        // AutoMoveOpen(drivetrain, slowXLeft[12], slowYLeft[12], slowRLeft[12], slowSLeft[12], invertA, invertD).ToPtr(),
        // intakeNextToHub(drivetrain,shooter,intaker,true).ToPtr()
        complexcommand->CloseStorageCommand(),
        complexcommand->GroundintakeprepareCommand(),
        AutoMoveOpen(drivetrain, AutoXLow[6], AutoYLow[6]-0.3, AutoRLow[6], AutoSLow[6], invertA, invertD).ToPtr().Until([drivetrain, invertA]{
          double r = drivetrain->GetState().Pose.Rotation().Degrees().value();
          return (!invertA&&r<80&&r>-80)||(invertA&&(r>100||r<-100));}),
        complexcommand->PassTrench(false).Until([drivetrain, invertA]{return (!invertA&&drivetrain->GetState().Pose.X().value()>AutoXLow[8])||(invertA&&drivetrain->GetState().Pose.X().value()<16.54-AutoXLow[8]);}),
        complexcommand->StartStorageCommand(),
        //AutoMoveOpen(drivetrain, AutoXLow[8], AutoYLow[8], AutoRLow[8], AutoSLow[8], invertA, invertD).ToPtr(),
        frc2::cmd::Either(
          frc2::cmd::Sequence(
            AutoMoveCircle(drivetrain, AutoXLow[9]-0.9, AutoYLow[9],0.7,0,false, AutoSLow[9],false,-90,false, invertA, invertD,0).ToPtr(),
            AutoMoveOpen(drivetrain, AutoXLow[10], AutoYLow[10]-3, AutoRLow[10], AutoSLow[10], invertA, invertD).ToPtr()
          ),
          frc2::cmd::Sequence(
            drivetrain->SetCoastCommand(),
            AutoMoveOpen(drivetrain, 11, 4, 180, 2.70, invertA, invertD).ToPtr()

          ),
          []{return !frc::SmartDashboard::GetBoolean("autoGoOppo",false);}
       )
       

      );
    }

//frc2::CommandPtr autos::ExampleAuto(CommandSwerveDrivetrain* drivetrain) {
//     frc::Pose2d start{units::meter_t{OutX[0]},units::meter_t{OutY[0]},frc::Rotation2d{units::degree_t{OutR[0]}}};
//   return frc2::cmd::Sequence(
//     frc2::cmd::RunOnce([drivetrain, start] {
//             drivetrain->ResetPose(start); 
//         }, {drivetrain}),
//     AutoMoveOpen(drivetrain,OutX[1],OutY[1],OutR[1],OutS[1], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[2],OutY[2],OutR[2],OutS[2], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[3],OutY[3],OutR[3],OutS[3], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[4],OutY[4],OutR[4],OutS[4], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[5],OutY[5],OutR[5],OutS[5], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[6],OutY[6],OutR[6],OutS[6], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[7],OutY[7],OutR[7],OutS[7], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[8],OutY[8],OutR[8],OutS[8], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[9],OutY[9],OutR[9],OutS[9], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[10],OutY[10],OutR[10],OutS[10], false,false).ToPtr(),
//     AutoMoveClosed(drivetrain,OutX[11],OutY[11],OutR[11],OutS[11], false,false).ToPtr()
// );
//  }
// frc2::CommandPtr autos::ExampleAuto1(CommandSwerveDrivetrain* drivetrain,ShooterSubsystem* shooter, 
//     FeederSubsystem* feeder, GroundIntakeSubsystem* intaker) {
//     double x=16.54-OutX[0];
//     double y=8.07-OutY[0];
//     double d=OutR[0];
//     d=0.0-d;
//     d=180-d;
//     frc::Pose2d start{units::meter_t{OutX[0]},units::meter_t{OutY[0]},frc::Rotation2d{units::degree_t{OutR[0]}}};
//   return frc2::cmd::Sequence(
//     frc2::cmd::RunOnce([drivetrain, start] {
//             drivetrain->ResetPose(start); 
//         }, {drivetrain}),
//     AutoMoveOpen(drivetrain,OutX[1],OutY[1],OutR[1],OutS[1], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[2],OutY[2],OutR[2],OutS[2], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[3],OutY[3],OutR[3],OutS[3], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[4],OutY[4],OutR[4],OutS[4], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[5],OutY[5],OutR[5],OutS[5], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[6],OutY[6],OutR[6],OutS[6], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[7],OutY[7],OutR[7],OutS[7], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[8],OutY[8],OutR[8],OutS[8], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[9],OutY[9],OutR[9],OutS[9], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[10],OutY[10],OutR[10],OutS[10], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[11],OutY[11],OutR[11],OutS[11], false,false).ToPtr(),
//     shooter->EnableShooter(),
//     AutoMoveClosed(drivetrain,OutX[12],OutY[12],OutR[12],OutS[12], false,false).ToPtr(),
//     feeder->HoldFeederVelocityCommandPtr(ShooterConstants::kBackwardFeederVelocity,ShooterConstants::kUpwardFeederVelocity),
//     frc2::cmd::Wait(units::second_t{2}),
//     feeder->HoldFeederVelocityCommandPtr(0,0),
//     shooter->DisableShooter(),
//     AutoMoveOpen(drivetrain,OutX[13],OutY[13],OutR[13],OutS[13], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[14],OutY[14],OutR[14],OutS[14], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[15],OutY[15],OutR[15],OutS[15], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[16],OutY[16],OutR[16],OutS[16], false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[17],OutY[17],OutR[17],OutS[17], false,false).ToPtr(),
//     AutoRealTimeAimDrive(drivetrain,shooter, feeder,OutX[18],OutY[18],false,false).ToPtr(),
//     AutoMoveOpen(drivetrain,OutX[19],OutY[19],OutR[19],OutS[19], false,false).ToPtr(),
//     AutoMoveClosed(drivetrain,OutX[20],OutY[20],OutR[20],OutS[20], false,false).ToPtr(),
//     AutoRealTimeAimDrive(drivetrain,shooter,feeder,OutX[21],OutY[21], false,false).ToPtr(),
//     AutoRealTimeAimDrive(drivetrain,shooter,feeder,OutX[22],OutY[22], false,false).ToPtr(),
//     AutoMoveClosed(drivetrain,OutX[23],OutY[23],OutR[23],OutS[23], false,false).ToPtr()
// );
//  }