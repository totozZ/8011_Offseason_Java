#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include "subsystems/CommandSwerveDrivetrain.h" 
using namespace subsystems;
class AutoMoveOpen : public frc2::CommandHelper<frc2::Command, AutoMoveOpen> {
public:
    /**
     * @param drivetrain 你的底盘子系统指针
     * @param targetWaypoints 目标点 (包含 X, Y, 和期望的朝向角)
     * @param targetVel 目标速度
     */
    AutoMoveOpen(CommandSwerveDrivetrain* drivetrain, double targetX, double targetY, double targetHeading, double targetVel, bool invert1, bool invert2);
    AutoMoveOpen(CommandSwerveDrivetrain* drivetrain, double targetX, double targetY, double targetHeading, double targetVel, bool invert1, bool invert2, double endSpeed);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    double targetRot=0;
    double initialDis=0;
    bool ChangeEnd=false;
    double endSpeed=0;
    bool invertA=false;
    bool invertD=false;
    double x=0;
    double y=0;
    double d=0;
    CommandSwerveDrivetrain* m_drivetrain;
    frc::Pose2d m_targetWaypoint;
    double angleToTarget=0;
    frc::PIDController m_PID{3.0, 0.0, 0.1};
    frc::Pose2d targetPose;
    swerve::requests::FieldCentricFacingAngle driveClosed;
    double targetVelocity=0;
    // 容差常量 (到达多近算作“完成”)
    const double kTranslationTolerance = 0.1; // 允许 10 厘米的误差
    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts;
};