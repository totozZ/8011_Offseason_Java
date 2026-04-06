// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/ExampleSubsystem.h"
#include "commands/ComplexCommand.h"
#include "commands/intakeNextToHub.h"
#include "commands/RealTimeAimDrive.h"
#include "commands/AutoMoveClosed.h"
#include "commands/AutoMoveOpen.h"
#include "commands/AutoMoveCircle.h"
#include "commands/ComplexCommand.h"
#include "commands/AutoRealTimeAimDrive.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/GroundIntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */                                  //1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19   20De 21   22   23   24
inline const std::vector<double> OutX={3.50,6.30,6.78,7.50,8.25,8.40,8.40,8.00,7.00,6.20,5.70,3.35,3.00,6.00,6.00,5.75,5.70,3.00,1.30,1.10,0.40,1.60,1.66,1.50};
inline const std::vector<double> OutY={5.50,5.50,6.70,6.70,6.10,5.80,2.20,1.30,1.30,1.75,2.20,2.30,2.30,2.20,5.20,5.70,5.70,5.70,5.70,6.00,6.00,4.76,3.78,3.70};
inline const std::vector<double> OutR={-45 ,-45 ,-45 ,-90 ,-90 ,-90 ,-90 ,60  ,42.6,42  ,42  ,42  ,42  ,60  ,90  ,-45 ,-43 ,-43 ,   0,0   ,0   ,0   ,0   ,0   };
inline const std::vector<double> OutS={0   ,3.0 ,3.0 ,3.1 ,3.2 ,3.4 ,3.5 ,3.1 ,3.3 ,3.2 ,3.1 ,3.0 ,3.0 ,3.0 ,3.1 ,2.0 ,2.0 ,3.0 ,   0,2   ,1.5 ,2   ,0   ,1   };
//                                                                             shoot          IaD  shoot
inline const std::vector<double> slowXLeft={4.40,5.70,7.00,7.80,7.80,5.80,3.20,1.50,1.10,0.50,0.50,2.20,9};
inline const std::vector<double> slowYLeft={7.30,7.30,7.10,5.90,5.10,5.40,5.40,5.70,7.00,6.90,5.60,5.40,5.50};
inline const std::vector<double> slowRLeft={0   ,0   ,-90 ,-90 ,-90 ,135 ,135 ,135 ,-110,-110,-110,135 ,135   };
inline const std::vector<double> slowSLeft={0   ,2.80,2.40,2.80,1.50,2.30,2.00,0.8 ,2.30,0.90,0.90,2.4 ,2.4 };
//                                                         r b c                sh      S   R    B    C E                    sh
inline const std::vector<double> AutoXDouble={3.60,5.80,7.00,7.80,7.80,5.80,3.20,5.90,5.90,3.50,7.20,7.80,7.80,7.00,5.80,3.20};
inline const std::vector<double> AutoYDouble={5.60,5.50,7.10,6.40,4.70,5.40,5.40,5.40,3.80,3.00,3.30,3.80,4.60,5.40,5.40,5.40};
inline const std::vector<double> AutoRDouble={-45 ,-135,-46 ,-90 ,-90 ,135 ,135 ,-135,-90 ,0   ,45  ,90  ,90  ,-45 ,180 ,135};
inline const std::vector<double> AutoSDouble={0   ,2.00,2.40,2.40,2.40,2.40,2.00,2.00,2.60,2.60,2.60,2.60,2.60,2.60,2.60,2.00};
//start from right                                                      back      8     9    10   11       
inline const std::vector<double> AutoXHP={4.40,5.70,7.00,7.80,7.80,5.80,3.20,1.50,0.60,0.80,3.20,8};
inline const std::vector<double> AutoYHP={0.70,0.70,0.90,2.10,3.30,2.50,2.50,0.80,0.60,0.80,2.40,2.4};
inline const std::vector<double> AutoRHP={45  ,135 ,46  ,90  ,90  ,-135,-135,-135,0   ,  0 ,-135 ,-135};
inline const std::vector<double> AutoSHP={0   ,2.60,2.60,2.60,2.30,2.60,2.00,2.40,1.50,2.00,2.0 ,2.4};
//                                             circle End              sh 去洞  CE       CE        CE    
inline const std::vector<double> AutoXLow={4.40,5.70,8.27,8.27,5.80,3.00,2.50,3.50,5.20,6.10,6.10,7.80,7.80,7.00,6.20,3.20};
inline const std::vector<double> AutoYLow={7.30,7.40,6.10,4.70,5.40,5.40,7.00,7.30,7.40,6.20,4.40,4.40,4.70,5.40,5.50,5.50};
inline const std::vector<double> AutoRLow={0   ,0   ,-90 ,-90 ,132  ,132 ,0   ,  0 ,0   ,-90 ,-90 ,90  ,90  ,130 ,180,130};
inline const std::vector<double> AutoSLow={0   ,2.80,2.80,1.40,2.60,2.00,2.20,2.20,2.40,2.40,2.40,2.40,2.40,2.40,2.40,2.00};
frc2::CommandPtr AutoSlowLeft(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand, bool invertA, bool invertD);

frc2::CommandPtr AutoDoubleLeft(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand, bool invertA, bool invertD);

frc2::CommandPtr AutoDoubleRight(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand);

frc2::CommandPtr AutoHP(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand, bool invertA, bool invertD);

frc2::CommandPtr AutoLow(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand, bool invertA, bool invertD);


frc2::CommandPtr AutoLowOnlyOneSide(CommandSwerveDrivetrain* m_drivetrain, ShooterSubsystem* shooter, 
    FeederSubsystem* feeder, GroundIntakeSubsystem* intake, ComplexCommand* complexcommand, bool invertA, bool invertD);

}  // namespace autos

