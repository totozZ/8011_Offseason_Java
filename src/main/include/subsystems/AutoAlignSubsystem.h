#pragma once

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>
#include <string>
#include <optional>
#include <array>

using namespace pathplanner;

namespace subsystems
{
    class AutoAlignSubsystem : public frc2::SubsystemBase
    {

    private:
        CommandSwerveDrivetrain *m_drivetrain;
        frc2::CommandXboxController *m_joystick;

        enum Position
        {
            LEFT,
            RIGHT,
            CENTER,
            NONE
        }; // 预期方位（LEFT, RIGHT, CENTER）

        frc::Pose2d nearest_tag_pos;            // 距离机器人最近的 Tag 位置
        std::array<frc::Pose2d, 6> all_tag_pos; // 所有当前联盟的 Tag 位置
        double distance;                        // 目标位置到 Tag 的距离（根据预期方位决定）

        frc::Pose2d current_pos; // 当前位置
        frc::Pose2d target_pos;  // 目标位置

        frc::Pose2d start_point;       // 路径起始点
        frc::Pose2d end_point;         // 路径结束点
        frc::Rotation2d current_angle; // 当前角度
        frc::Rotation2d target_angle;  // 目标角度
        frc::Rotation2d start_heading; // 起始时轮子的朝向
        frc::Rotation2d end_heading;   // 结束时的方向
        double vx;                     // X 方向的线性速度
        double vy;                     // Y 方向的线性速度
        double v;                      // 线性速度

        bool has_command = false; // Command 相关，为 False 时没有正在执行的 Command，为 True 时有正在执行的 Command

        bool is_red = false;

        int path_generation_failed = 0; // 路径生成失败的次数

        std::optional<frc2::CommandPtr> vision_follow_command; // 当前正在执行的对齐 Command

        nt::NetworkTableInstance m_inst;

        std::array<nt::BooleanSubscriber, 12> m_clientSubscribers;

    public:
        AutoAlignSubsystem(CommandSwerveDrivetrain *drivetrain, frc2::CommandXboxController *joystick);
        void Periodic() override;
        void getJoystickInput();
        frc::Pose2d getNearestTag();
        frc::Pose2d calculateTargetPos(Position position);
        std::shared_ptr<PathPlannerPath> generatePath(frc::Pose2d end_point, double max_speed, double max_acc);
        frc2::CommandPtr followPathCommand(Position position, double max_speed = 2.5, double max_acc = 2);
        Position checkAvailable();
        int getNearestTagId();
    };
}