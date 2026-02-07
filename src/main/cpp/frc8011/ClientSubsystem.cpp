#include "frc8011/ClientSubsystem.h"
using namespace subsystems;

ClientSubsystem::ClientSubsystem(subsystems::CommandSwerveDrivetrain *driveSubsystem):
m_drivesubsystem(driveSubsystem)
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    m_table = inst.GetTable("client");
    targetPoseSub = m_table->GetDoubleArrayTopic("targetPose").Subscribe(std::vector<double>{0.0, 0.0, 0.0});
    levelSub = m_table->GetIntegerTopic("targetLevel").Subscribe(2);
    is_completedPub = m_table->GetBooleanTopic("isCompleted").Publish();
}

void ClientSubsystem::Periodic()
{
try
{
    getLevel();
    getTargetPose();

    std::vector<double> targetData = {
            targetPose.Translation().X().value(),
            targetPose.Translation().Y().value(),
            targetPose.Rotation().Degrees().value(),
            static_cast<double>(GetLevel())};
    frc::SmartDashboard::PutNumberArray("client_target_data", targetData);
    
  frc::SmartDashboard::PutNumber("client_level",
                                 GetLevel());




}
catch(const std::exception& e)
{
    frc::SmartDashboard::PutString("ClientSubsystem Periodic Failed:", e.what());
}


  


}
