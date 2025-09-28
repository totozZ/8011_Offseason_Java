#include "subsystems/AutoAlignSubsystem.h"

using namespace subsystems;

AutoAlignSubsystem::AutoAlignSubsystem(CommandSwerveDrivetrain *drivetrain, frc2::CommandXboxController *joystick)
    : m_drivetrain(drivetrain), m_joystick(joystick)
{
    // 根据 DriverStation 的 Alliance 判断使用哪些 Tag
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        all_tag_pos = AutoAlignConstants::RED_TAG_POS;
        is_red = true;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        all_tag_pos = AutoAlignConstants::BLUE_TAG_POS;
        is_red = false;
    }

    m_inst = nt::NetworkTableInstance::GetDefault();

    for (int i = 0; i < 12; i++)
    {
        std::string topic_name = "Client/4/" + std::to_string(i);
        m_clientSubscribers[i] = m_inst.GetBooleanTopic(topic_name).Subscribe(false);
    }
}

void AutoAlignSubsystem::Periodic()
{
    // 获取手柄输入（Left Bumper, Right Bumper, POVLeft），执行对应的 Command
    getJoystickInput();
}

// 获取手柄输入（Left Bumper, Right Bumper, POVLeft），执行对应的 Command
void AutoAlignSubsystem::getJoystickInput()
{
    if (m_joystick->GetHID().GetLeftBumperButton() || m_joystick->GetHID().GetRightBumperButton() || m_joystick->GetHID().GetPOV() == 180 || m_joystick->GetHID().GetAButton())
    {
        if (!has_command)
        {
            if (m_joystick->GetHID().GetAButton())
            {
                frc::SmartDashboard::PutString("texting", "2");
                Position result = checkAvailable();
                if (result != Position::NONE)
                {
                    vision_follow_command = followPathCommand(result);
                    vision_follow_command->Schedule();
                }
                else
                {
                    frc::SmartDashboard::PutString("Available Position", "No valid position");
                }
            }

            // 对齐右边 Reef
            if (m_joystick->GetHID().GetRightBumperButton())
            {
                vision_follow_command = followPathCommand(Position::RIGHT);
                vision_follow_command->Schedule();
            }
            // 对齐左边 Reef
            else if (m_joystick->GetHID().GetLeftBumperButton())
            {
                vision_follow_command = followPathCommand(Position::LEFT);
                vision_follow_command->Schedule();
            }
            // 对齐中间
            else if (m_joystick->GetHID().GetPOV() == 180)
            {
                vision_follow_command = followPathCommand(Position::CENTER);
                vision_follow_command->Schedule();
            }
            // has_command 为 true 时不会再次执行 Command，防止多个 Command 同时执行
            if (vision_follow_command && vision_follow_command->IsScheduled())
            {
                has_command = true;
            }
        }
    }
    else
    {
        // 如果 Command 正在执行，取消当前 Command
        if (vision_follow_command && vision_follow_command->IsScheduled())
        {
            frc2::CommandScheduler::GetInstance().Cancel(vision_follow_command.value());
        }
        has_command = false;
        // 松开按键时重置 has_command
        vision_follow_command.reset();
    }
}

/**
 * \brief 执行对应的 Command
 *
 * \param position 预期方位（LEFT, RIGHT, CENTER）
 * \param max_speed   最大速度（m/s），默认2.5m/s，用于PathPlanner限制
 * \param max_acc 最大加速度（m/s²），默认2m/s²，用于PathPlanner限制
 */
frc2::CommandPtr AutoAlignSubsystem::followPathCommand(Position position, double max_speed, double max_acc)
{
    // 根据预期方位（LEFT, RIGHT, CENTER）计算具体位置
    target_pos = calculateTargetPos(position);

    // 生成路径
    auto goal_path = generatePath(target_pos, max_speed, max_acc);
    // 如果生成的路径不符合条件，generatePath() 会传回 nullPtr，不执行路径
    if (goal_path == nullptr)
    {
        return frc2::cmd::RunOnce([this]
                                  { frc::SmartDashboard::PutNumber("PathPlanner path generation failed", path_generation_failed++); });
    }
    else
    {
        return frc2::cmd::Sequence(AutoBuilder::followPath(goal_path));
    }
}

// 根据预期方位（LEFT, RIGHT, CENTER）计算具体位置
frc::Pose2d AutoAlignSubsystem::calculateTargetPos(Position position)
{
    switch (position)
    {
    case Position::LEFT:
        distance = AutoAlignConstants::LEFT_TO_TAG_POS.value();
        break;
    case Position::RIGHT:
        distance = AutoAlignConstants::RIGHT_TO_TAG_POS.value();
        break;
    case Position::CENTER:
        distance = AutoAlignConstants::CENTER_TO_TAG_POS.value();
        break;
    default:
        break;
    }
    // 获取最近的 Tag 位置
    nearest_tag_pos = getNearestTag();
    return frc::Pose2d{
        units::meter_t{nearest_tag_pos.Translation().X().value() -
                       std::cos(nearest_tag_pos.Rotation().Radians().value()) * RobotInfoConstants::CENTER_TO_BUMPER.value() - std::sin(nearest_tag_pos.Rotation().Radians().value()) * distance},

        units::meter_t{nearest_tag_pos.Translation().Y().value() -
                       std::sin(nearest_tag_pos.Rotation().Radians().value()) * RobotInfoConstants::CENTER_TO_BUMPER.value() + std::cos(nearest_tag_pos.Rotation().Radians().value()) * distance},

        frc::Rotation2d{
            units::degree_t{nearest_tag_pos.Rotation().Degrees().value()}}};
}

// 获取最近的 Tag 位置
frc::Pose2d AutoAlignSubsystem::getNearestTag()
{
    return m_drivetrain->GetState().Pose.Nearest(all_tag_pos);
}

int AutoAlignSubsystem::getNearestTagId()
{
    frc::Pose2d nearest_pose = getNearestTag();

    auto it = std::find(all_tag_pos.begin(), all_tag_pos.end(), nearest_pose);
    int index = std::distance(all_tag_pos.begin(), it);

    if (is_red)
    {
        return RobotConstants::RED_VALID_APRILTAGS[index];
    }
    else
    {
        return RobotConstants::BLUE_VALID_APRILTAGS[index];
    }
}

// 生成路径
std::shared_ptr<PathPlannerPath> AutoAlignSubsystem::generatePath(frc::Pose2d end_point, double max_speed, double max_acc)
{
    current_pos = m_drivetrain->GetState().Pose;

    // 如果目标位置和当前位置过于接近，不生成路径
    if (std::abs(end_point.Translation().X().value() - current_pos.Translation().X().value()) < 0.05 && std::abs(end_point.Translation().Y().value() - current_pos.Translation().Y().value()) < 0.05)
    {
        return nullptr;
    }

    // current_angle = current_pos.Rotation();
    target_angle = end_point.Rotation();

    // 计算线性速度
    // vx = m_drivetrain->GetState().Speeds.vx();
    // vy = m_drivetrain->GetState().Speeds.vy();
    // v = sqrt(vx * vx + vy * vy);

    // 使用线性速度计算当前轮子的朝向，作为起始方向
    // start_heading = frc::Rotation2d{units::radian_t{atan2(vy, vx)}};

    frc::Pose2d center_point = frc::Pose2d{(current_pos.Translation().X().value() + end_point.Translation().X().value()) / 2 * 1_m, (current_pos.Translation().Y().value() + end_point.Translation().Y().value()) / 2 * 1_m, target_angle};

    // 使用起始点和结束点创建 poses
    std::vector<frc::Pose2d>
        poses = {
            current_pos,
            center_point,
            end_point};

    // 使用 poses 创建 waypoints，用于传入 path
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

    // 路径限制
    PathConstraints constraints(max_speed * 1_mps, max_acc * 1_mps_sq, 540_deg_per_s, 980_deg_per_s_sq);

    // 创建路径
    auto path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        std::nullopt,
        GoalEndState(0_mps, target_angle) // 默认结束速度永远为 0
    );

    // 防止路径在正确的坐标下被翻转
    path->preventFlipping = true;

    frc::SmartDashboard::PutNumberArray("start_point", std::vector<double>{current_pos.Translation().X().value(), current_pos.Translation().Y().value(), current_pos.Rotation().Degrees().value()});
    frc::SmartDashboard::PutNumberArray("end_point", std::vector<double>{end_point.Translation().X().value(), end_point.Translation().Y().value(), end_point.Rotation().Degrees().value()});

    return path;
}

AutoAlignSubsystem::Position AutoAlignSubsystem::checkAvailable()
{
    int nearest_tag_id = getNearestTagId();
    std::array<int, 2> client_index = {12, 12};

    frc::SmartDashboard::PutString("texting", "1");

    switch (nearest_tag_id)
    {
    case 10:
    case 21:
        client_index = {0, 1};
        break;
    case 9:
    case 22:
        client_index = {2, 3};
        break;
    case 8:
    case 17:
        client_index = {4, 5};
        break;
    case 7:
    case 18:
        client_index = {6, 7};
        break;
    case 6:
    case 19:
        client_index = {8, 9};
        break;
    case 11:
    case 20:
        client_index = {10, 11};
        break;
    default:
        break;
    }

    if (client_index[0] != 12 && m_clientSubscribers[client_index[0]].Get(false))
    {
        return Position::RIGHT;
    }
    else if (client_index[1] != 12 && m_clientSubscribers[client_index[1]].Get(false))
    {
        return Position::LEFT;
    }
    else
    {
        return Position::RIGHT;
    }
}