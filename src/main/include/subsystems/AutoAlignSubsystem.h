#pragma once

// #include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
// #include <ctre/phoenix6/swerve/requests/ApplyRobotSpeeds.hpp>
#include "generated/TunerConstants.h"
#include "Constants.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <string>
#include <optional>
#include <array>
#include <cmath>
#include <units/length.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StringTopic.h>

// using namespace pathplanner;

namespace subsystems
{
    class AutoAlignSubsystem : public frc2::SubsystemBase
    {

    private:
        CommandSwerveDrivetrain *m_drivetrain;
        frc2::CommandXboxController *m_joystick;

        frc::Pose2d nearest_tag_pos;            // 距离机器最近的 Tag 位置
        std::array<frc::Pose2d, 6> all_tag_pos; // 所有当前联盟的 Tag 位置
        frc::Pose2d current_pos;                // 当前位置
        frc::Pose2d target_pos;                 // 目标位置

        // frc::Pose2d start_point;       // 路径起始点
        // frc::Pose2d end_point;         // 路径结束点
        // frc::Rotation2d current_angle; // 当前角度
        // frc::Rotation2d target_angle;  // 目标角度
        // frc::Rotation2d start_heading; // 起始时轮子的朝向
        // frc::Rotation2d end_heading;   // 结束时的方向
        // double vx;                     // X 方向的线性速度
        // double vy;                     // Y 方向的线性速度
        // double v;                      // 线性速度

        // int path_generation_failed = 0; // 路径生成失败的次数
        // std::optional<frc2::CommandPtr> vision_follow_command; // 当前正在执行的对齐 Command

        frc::PIDController m_xController{AutoAlignConstants::X_KP, AutoAlignConstants::X_KI, AutoAlignConstants::X_KD};                 // X 方向 PID 控制器
        frc::PIDController m_yController{AutoAlignConstants::Y_KP, AutoAlignConstants::Y_KI, AutoAlignConstants::Y_KD};                 // Y 方向 PID 控制器
        frc::PIDController m_thetaController{AutoAlignConstants::THETA_KP, AutoAlignConstants::THETA_KI, AutoAlignConstants::THETA_KD}; // 角度 PID 控制器

        std::optional<frc2::CommandPtr> pid_align_command; // 当前正在执行的PID对齐Command

        nt::NetworkTableInstance m_inst;
        std::array<nt::BooleanSubscriber, 12> m_clientSubscribers;

        std::shared_ptr<nt::NetworkTable> PIDTable = nt::NetworkTableInstance::GetDefault().GetTable("PIDControllerSubsystem");
        std::shared_ptr<nt::NetworkTable> FMSTable = nt::NetworkTableInstance::GetDefault().GetTable("FMSInfo");

        nt::StructPublisher<frc::Pose2d> start_pose = PIDTable->GetStructTopic<frc::Pose2d>("StartPose").Publish();
        nt::StructPublisher<frc::Pose2d> end_pose = PIDTable->GetStructTopic<frc::Pose2d>("EndPose").Publish();
        nt::StructPublisher<frc::Pose2d> idel_end_pose = PIDTable->GetStructTopic<frc::Pose2d>("IdelEndPose").Publish();
        nt::StructPublisher<frc::ChassisSpeeds> current_speeds = PIDTable->GetStructTopic<frc::ChassisSpeeds>("Speed").Publish();
        nt::StructArrayPublisher<frc::Pose2d> current_tag_pos = PIDTable->GetStructArrayTopic<frc::Pose2d>("TagPose").Publish();

        nt::StringPublisher PID_state = PIDTable->GetStringTopic("PIDState").Publish();

        bool has_command = false; // Command 相关，为 False 时没有正在执行的 Command，为 True 时有正在执行的 Command
        bool is_red = false;
        bool isContinuous = false; // 连续移动标志，为 true 时不减速

    public:
        enum Position
        {
            LEFT,
            RIGHT,
            CENTER,
            NONE
        }; // 预期方位（LEFT, RIGHT, CENTER）

        AutoAlignSubsystem(CommandSwerveDrivetrain *drivetrain, frc2::CommandXboxController *joystick);
        frc::Pose2d getNearestTag();
        frc::Pose2d calculateTargetPos(Position position);

        // PathPlanner相关方法已注释
        // std::shared_ptr<PathPlannerPath> generatePath(frc::Pose2d end_point, double max_speed, double max_acc);
        // frc2::CommandPtr followPathCommand(Position position, double max_speed = 2.5, double max_acc = 2);

        // PID控制相关方法
        frc2::CommandPtr PIDAlignCommand(Position position, bool isContinuous = false);
        frc::ChassisSpeeds calculatePIDSpeeds();
        void reset();

        Position checkAvailable();
    };
}