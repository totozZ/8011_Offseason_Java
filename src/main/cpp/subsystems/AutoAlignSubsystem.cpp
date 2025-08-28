#include "subsystems/AutoAlignSubsystem.h"

using namespace subsystems;

AutoAlignSubsystem::AutoAlignSubsystem(CommandSwerveDrivetrain *drivetrain, frc2::CommandXboxController *joystick)
    : m_drivetrain(drivetrain), m_joystick(joystick)
{
    // 根据DriverStation的Alliance判断使用哪些Tag
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        all_tag_pos = AutoAlignConstants::RED_TAG_POS;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        all_tag_pos = AutoAlignConstants::BLUE_TAG_POS;
    }
}

void AutoAlignSubsystem::Periodic()
{
    // // 获取手柄输入（Left Bumper, Right Bumper, POVLeft）
    getJoystickInput();
}

// 获取手柄输入（Left Bumper, Right Bumper, POVLeft）
void AutoAlignSubsystem::getJoystickInput()
{
    if ((m_joystick->LeftBumper().Get() || m_joystick->RightBumper().Get() || m_joystick->POVLeft().Get()) && !has_command)
    {
        // 对齐左边Reef
        if (m_joystick->LeftBumper().Get())
        {
            vision_follow_command = followPathCommand(Position::LEFT);
            vision_follow_command->Schedule();
        }

        // 对齐右边Reef
        if (m_joystick->RightBumper().Get())
        {
            vision_follow_command = followPathCommand(Position::RIGHT);
            vision_follow_command->Schedule();
        }

        // 对齐中间
        if (m_joystick->POVLeft().Get())
        {
            vision_follow_command = followPathCommand(Position::CENTER);
            vision_follow_command->Schedule();
        }

        // has_command为true时不会再次执行Command，防止多个Command同时执行
        has_command = true;
    }
    else
    {
        // 如果Command正在执行，取消当前Command
        if (vision_follow_command && vision_follow_command->IsScheduled())
        {
            frc2::CommandScheduler::GetInstance().Cancel(vision_follow_command.value());
        }
        // 松开按键时重置has_command
        has_command = false;
        vision_follow_command.reset();
    }
}

/**
 * \brief 执行对应的Command
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
    // 如果生成的路径不符合条件，generatePath()会传回nullPtr，不执行路径
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
    // 获取最近的Tag位置
    nearest_tag_pos = getNearestTag();
    return frc::Pose2d{
        units::meter_t{nearest_tag_pos.Translation().X().value() -
                       std::cos(nearest_tag_pos.Rotation().Radians().value()) * RobotInfoConstants::CENTER_TO_BUMPER.value() - std::sin(nearest_tag_pos.Rotation().Radians().value()) * distance},

        units::meter_t{nearest_tag_pos.Translation().Y().value() -
                       std::sin(nearest_tag_pos.Rotation().Radians().value()) * RobotInfoConstants::CENTER_TO_BUMPER.value() + std::cos(nearest_tag_pos.Rotation().Radians().value()) * distance},

        frc::Rotation2d{
            units::degree_t{nearest_tag_pos.Rotation().Degrees().value()}}};
}

// 获取最近的Tag位置
frc::Pose2d AutoAlignSubsystem::getNearestTag()
{
    return m_drivetrain->GetState().Pose.Nearest(all_tag_pos);
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

    // 如果Rotation2d为{0, 0}，认为时PathPlanner错误，不生产路径
    if (end_point.Rotation() == frc::Rotation2d{0, 0} || current_pos.Rotation() == frc::Rotation2d{0, 0})
    {
        return nullptr;
    }

    current_angle = current_pos.Rotation();
    target_angle = end_point.Rotation();

    // 计算线性速度
    vx = m_drivetrain->GetState().Speeds.vx();
    vy = m_drivetrain->GetState().Speeds.vy();
    v = sqrt(vx * vx + vy * vy);

    // 使用线性速度计算当前轮子的朝向，作为起始方向
    start_heading = frc::Rotation2d{units::radian_t{atan2(vy, vx)}};

    // 整合起始点
    start_point = frc::Pose2d{
        current_pos.Translation(),
        start_heading};

    // 使用起始点和结束点创建poses
    std::vector<frc::Pose2d> poses = {
        start_point,
        end_point};

    // 使用poses创建waypoints，用于传入path
    std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

    // 路径限制
    PathConstraints constraints(max_speed * 1_mps, max_acc * 1_mps_sq, 540_deg_per_s, 980_deg_per_s_sq);

    // 创建路径
    auto path = std::make_shared<PathPlannerPath>(
        waypoints,
        constraints,
        IdealStartingState(v * 1_mps, start_heading),
        GoalEndState(0_mps, target_angle) // 默认结束速度永远为0
    );

    // 防止路径在正确的坐标下被翻转
    path->preventFlipping = true;

    frc::SmartDashboard::PutNumberArray("start_point", std::vector<double>{start_point.Translation().X().value(), start_point.Translation().Y().value(), start_point.Rotation().Degrees().value()});
    frc::SmartDashboard::PutNumberArray("end_point", std::vector<double>{end_point.Translation().X().value(), end_point.Translation().Y().value(), end_point.Rotation().Degrees().value()});

    return path;
}
