#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/CommandSwerveDrivetrain.h"
namespace subsystems
{
    class ClientSubsystem : public frc2::SubsystemBase
    {
    public:
        explicit ClientSubsystem(subsystems::CommandSwerveDrivetrain *driveSubsystem);
        void Periodic() override;



        frc::Pose2d GetTargetPose()
        {
            return targetPose;
        }
        
        int GetLevel()
        {
            return level;
        }  

        void PubIsCompleted() { 
            is_completedPub.Set(true); 
        };

        void PubRobotInit(bool init) {
            m_table->GetBooleanTopic("init").Publish().Set(init);
        }

    private:

        std::shared_ptr<nt::NetworkTable> m_table;

        nt::DoubleArraySubscriber targetPoseSub;
        nt::IntegerSubscriber levelSub;
        nt::BooleanPublisher is_completedPub;
        frc::Pose2d targetPose;
        int level;
        bool is_completed;

        subsystems::CommandSwerveDrivetrain *m_drivesubsystem;

        void getLevel()
        {
            auto levelData = levelSub.Get();
            level = levelData;
        };

        void getTargetPose()
        {
            auto poseData = targetPoseSub.Get();
            targetPose = frc::Pose2d(units::meter_t{poseData[0]},
                                     units::meter_t{poseData[1]},
                                     frc::Rotation2d(units::degree_t{poseData[2]}));
        }
    };
}