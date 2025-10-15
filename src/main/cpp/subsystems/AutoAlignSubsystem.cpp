#include "subsystems/AutoAlignSubsystem.h"

using namespace subsystems;

AutoAlignSubsystem::AutoAlignSubsystem(CommandSwerveDrivetrain *drivetrain, frc2::CommandXboxController *joystick)
    : m_drivetrain(drivetrain), m_joystick(joystick)
{
    // 根据 DriverStation 的 Alliance 判断使用哪些 Tag
    if (FMSTable->GetBoolean("IsRedAlliance", true))
    {
        all_tag_pos = TagConstants::RED_TAG_POS;
        is_red = true;
    }
    else
    {
        all_tag_pos = TagConstants::BLUE_TAG_POS;
        is_red = false;
    }

    current_tag_pos.Set(all_tag_pos);

    // 初始化PID控制器
    m_xController.SetTolerance(AutoAlignConstants::X_TOLERANCE);
    m_yController.SetTolerance(AutoAlignConstants::Y_TOLERANCE);
    m_thetaController.SetTolerance(AutoAlignConstants::THETA_TOLERANCE);
    m_thetaController.EnableContinuousInput(-180, 180);

    // m_inst = nt::NetworkTableInstance::GetDefault();

    // for (int i = 0; i < 12; i++)
    // {
    //     std::string topic_name = "Client/4/" + std::to_string(i);
    //     m_clientSubscribers[i] = m_inst.GetBooleanTopic(topic_name).Subscribe(false);
    // }
}

/**
 * \brief 使用PID控制器执行对齐Command
 *
 * \param position 预期方位（LEFT, RIGHT, CENTER）
 * \param isContinuous 结束速度为 0 还是为满速，为 true 时为满速，为 false 时为 0
 */
frc2::CommandPtr AutoAlignSubsystem::PIDAlignCommand(Position position, bool isContinuous)
{
    return frc2::cmd::RunOnce([this, position, isContinuous]
                              { 
                                PID_state.Set("Calculating Target Pose");
                                current_pos = m_drivetrain->GetState().Pose;
                                start_pose.Set(current_pos);
                                target_pos = calculateTargetPos(position); 
                                // target_pos = current_pos.TransformBy(AutoAlignConstants::TEST_TRANSFORM);
                                idel_end_pose.Set(target_pos);
                                this->isContinuous = isContinuous; })
        .AndThen(frc2::cmd::Run([this]
                                {
                                    PID_state.Set("Running");
                                    frc::ChassisSpeeds speeds = calculatePIDSpeeds(); // 计算PID速度
                                    current_speeds.Set(speeds);
                                    static swerve::requests::ApplyRobotSpeeds kApply;
                                    m_drivetrain->SetControl(kApply.WithSpeeds(speeds)); }, {m_drivetrain}))
        .Until([this]
               { return m_xController.AtSetpoint() && m_yController.AtSetpoint() && m_thetaController.AtSetpoint(); }) // 到了 Setpoint 就结束
        .FinallyDo([this]
                   {
                PID_state.Set("Finished");
                this->isContinuous = false; });
}

// 根据预期方位（LEFT, RIGHT, CENTER）计算具体位置
frc::Pose2d AutoAlignSubsystem::calculateTargetPos(Position position)
{
    units::meter_t distance = 0_m;

    switch (position)
    {
    case Position::LEFT:
        distance = TagConstants::LEFT_TO_TAG_POS;
        break;
    case Position::RIGHT:
        distance = TagConstants::RIGHT_TO_TAG_POS;
        break;
    case Position::CENTER:
        distance = TagConstants::CENTER_TO_TAG_POS;
        break;
    default:
        break;
    }

    // 获取最近的 Tag 位置
    nearest_tag_pos = getNearestTag();

    return nearest_tag_pos.TransformBy(
        frc::Transform2d{
            frc::Translation2d{-RobotConstants::CENTER_TO_BUMPER, distance},
            frc::Rotation2d{0_deg}});
}

// 获取最近的 Tag 位置
frc::Pose2d AutoAlignSubsystem::getNearestTag()
{

    return m_drivetrain->GetState().Pose.Nearest(all_tag_pos);
}

/**
 * \brief 计算PID控制速度
 *
 * \return 计算出的底盘速度
 */
frc::ChassisSpeeds AutoAlignSubsystem::calculatePIDSpeeds()
{
    current_pos = m_drivetrain->GetState().Pose;

    double current_X = current_pos.Translation().X().value();
    double current_Y = current_pos.Translation().Y().value();
    double current_theta = current_pos.Rotation().Degrees().value();

    double target_X = target_pos.Translation().X().value();
    double target_Y = target_pos.Translation().Y().value();
    double target_theta = target_pos.Rotation().Degrees().value();

    double vx, vy;
    double omega = m_thetaController.Calculate(current_theta, target_theta); // 角速度始终使用PID

    if (isContinuous)
    {
        // 使用最大速度而不是PID计算的速度
        double x_error = target_X - current_X;
        double y_error = target_Y - current_Y;
        double distance_error = std::hypot(x_error, y_error);

        if (distance_error != 0)
        {
            // 计算朝向目标点的单位向量
            double direction_x = x_error / distance_error;
            double direction_y = y_error / distance_error;

            // 使用最大线性速度
            vx = direction_x * AutoAlignConstants::MAX_LINEAR_SPEED;
            vy = direction_y * AutoAlignConstants::MAX_LINEAR_SPEED;
        }
        else
        {
            vx = 0;
            vy = 0;
        }
    }
    else
    {
        // 正常PID模式
        vx = m_xController.Calculate(current_X, target_X); // X 速度
        vy = m_yController.Calculate(current_Y, target_Y); // Y 速度
    }

    // 确保不超速
    vx = std::clamp(vx, -AutoAlignConstants::MAX_LINEAR_SPEED, AutoAlignConstants::MAX_LINEAR_SPEED);
    vy = std::clamp(vy, -AutoAlignConstants::MAX_LINEAR_SPEED, AutoAlignConstants::MAX_LINEAR_SPEED);
    omega = std::clamp(omega, -AutoAlignConstants::MAX_ANGULAR_SPEED, AutoAlignConstants::MAX_ANGULAR_SPEED);

    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{vx},
        units::meters_per_second_t{vy},
        units::degrees_per_second_t{omega},
        current_pos.Rotation());
}

void AutoAlignSubsystem::reset()
{
    m_xController.Reset();
    m_yController.Reset();
    m_thetaController.Reset();
    end_pose.Set(m_drivetrain->GetState().Pose);
    idel_end_pose.Set({0_m, 0_m, 0_deg});
}

// /**
//  * \brief 执行对应的 Command
//  *
//  * \param position 预期方位（LEFT, RIGHT, CENTER）
//  * \param max_speed   最大速度（m/s），默认2.5m/s，用于PathPlanner限制
//  * \param max_acc 最大加速度（m/s²），默认2m/s²，用于PathPlanner限制
//  */
// frc2::CommandPtr AutoAlignSubsystem::followPathCommand(Position position, double max_speed, double max_acc)
// {
//     // 根据预期方位（LEFT, RIGHT, CENTER）计算具体位置
//     target_pos = calculateTargetPos(position);

//     // 生成路径
//     auto goal_path = generatePath(target_pos, max_speed, max_acc);
//     // 如果生成的路径不符合条件，generatePath() 会传回 nullPtr，不执行路径
//     if (goal_path == nullptr)
//     {
//         return frc2::cmd::RunOnce([this]
//                                   { frc::SmartDashboard::PutNumber("PathPlanner path generation failed", path_generation_failed++); });
//     }
//     else
//     {
//         return frc2::cmd::Sequence(AutoBuilder::followPath(goal_path));
//     }
// }

// // 生成路径
// std::shared_ptr<PathPlannerPath> AutoAlignSubsystem::generatePath(frc::Pose2d end_point, double max_speed, double max_acc)
// {
//     // 如果目标位置和当前位置过于接近，不生成路径
//     if (std::abs(end_point.Translation().X().value() - current_pos.Translation().X().value()) < 0.05 && std::abs(end_point.Translation().Y().value() - current_pos.Translation().Y().value()) < 0.05)
//     {
//         return nullptr;
//     }

//     target_angle = end_point.Rotation();

//     // 计算线性速度
//     vx = m_drivetrain->GetState().Speeds.vx();
//     vy = m_drivetrain->GetState().Speeds.vy();

//     if (vx == 0 && vy == 0)
//     {
//         current_angle = current_pos.Rotation();
//     }
//     else
//     {
//         v = sqrt(vx * vx + vy * vy);
//         // 使用线性速度计算当前轮子的朝向，作为起始方向
//         current_angle = frc::Rotation2d{units::radian_t{atan2(vy, vx)}};
//     }

//     // 使用起始点和结束点创建 poses
//     std::vector<frc::Pose2d>
//         poses = {
//             current_pos,
//             end_point};

//     // 使用 poses 创建 waypoints，用于传入 path
//     std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

//     // 路径限制
//     PathConstraints constraints(max_speed * 1_mps, max_acc * 1_mps_sq, 540_deg_per_s, 980_deg_per_s_sq);

//     // 创建路径
//     auto path = std::make_shared<PathPlannerPath>(
//         waypoints,
//         constraints,
//         IdealStartingState(v * 1_mps, current_angle),
//         GoalEndState(0_mps, target_angle) // 默认结束速度永远为 0
//     );

//     // 防止路径在正确的坐标下被翻转
//     path->preventFlipping = true;

//     frc::SmartDashboard::PutNumberArray("start_point", std::vector<double>{current_pos.Translation().X().value(), current_pos.Translation().Y().value(), current_pos.Rotation().Degrees().value()});
//     frc::SmartDashboard::PutNumberArray("end_point", std::vector<double>{end_point.Translation().X().value(), end_point.Translation().Y().value(), end_point.Rotation().Degrees().value()});

//     return path;
// }

// int AutoAlignSubsystem::getNearestTagId()
// {
//     frc::Pose2d nearest_pose = getNearestTag();

//     auto it = std::find(all_tag_pos.begin(), all_tag_pos.end(), nearest_pose);
//     int index = std::distance(all_tag_pos.begin(), it);

//     if (is_red)
//     {
//         return TagConstants::RED_VALID_TAGS[index];
//     }
//     else
//     {
//         return TagConstants::BLUE_VALID_TAGS[index];
//     }
// }

// AutoAlignSubsystem::Position AutoAlignSubsystem::checkAvailable()
// {
//     int nearest_tag_id = getNearestTagId();
//     std::array<int, 2> client_index = {12, 12};

//     frc::SmartDashboard::PutString("texting", "1");

//     switch (nearest_tag_id)
//     {
//     case 10:
//     case 21:
//         client_index = {0, 1};
//         break;
//     case 9:
//     case 22:
//         client_index = {2, 3};
//         break;
//     case 8:
//     case 17:
//         client_index = {4, 5};
//         break;
//     case 7:
//     case 18:
//         client_index = {6, 7};
//         break;
//     case 6:
//     case 19:
//         client_index = {8, 9};
//         break;
//     case 11:
//     case 20:
//         client_index = {10, 11};
//         break;
//     default:
//         break;
//     }

//     if (client_index[0] != 12 && m_clientSubscribers[client_index[0]].Get(false))
//     {
//         return Position::RIGHT;
//     }
//     else if (client_index[1] != 12 && m_clientSubscribers[client_index[1]].Get(false))
//     {
//         return Position::LEFT;
//     }
//     else
//     {
//         return Position::RIGHT;
//     }
// }