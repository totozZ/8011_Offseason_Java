#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/PIDController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include <units/velocity.h>

using namespace subsystems;

class AutoMoveCircle : public frc2::CommandHelper<frc2::Command, AutoMoveCircle> {
public:
    // 新增 faceTravelDir：为 true 时自动将车头对准切线轨迹，忽略 targetHeadingDeg
    AutoMoveCircle(CommandSwerveDrivetrain* drivetrain, 
                   double centerX, double centerY, double radius, 
                   double targetAngleDeg, bool isCCW, 
                   double targetVel, bool stopAtEnd, 
                   double targetHeadingDeg, bool faceTravelDir, 
                   bool invertA, bool invertD, double angleOffset);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    CommandSwerveDrivetrain* m_drivetrain;
    
    // 几何与参数
    double angleOff;//在面向行进方向时角度offset
    double m_Cx, m_Cy, m_radius;
    double m_targetAngleDeg;
    double m_targetVel, m_targetHeadingDeg;
    bool m_isCCW, m_stopAtEnd, m_faceTravelDir, m_invertA, m_invertD;
    int m_direction; // 1 (CCW) 或 -1 (CW)
    
    // 角度积分与终点
    frc::Rotation2d m_lastAngleFromCenter;
    double m_accumulatedDegrees;
    double m_totalDegreesNeeded;
    frc::Translation2d m_targetPoint; // 物理终点坐标

    // PID 控制器
    frc::PIDController m_radialPidX{2, 0.1, 0.1}; 
    frc::PIDController m_radialPidY{2, 0.1, 0.1}; 
    frc::PIDController m_distPid{3, 0.1, 0.1};    

    swerve::requests::FieldCentricFacingAngle driveClosed;
};