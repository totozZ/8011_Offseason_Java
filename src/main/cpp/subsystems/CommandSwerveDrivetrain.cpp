#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <iostream>
#include "simulation/SimulationManager.h"

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this]
        { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](frc::Pose2d const &pose)
        { return ResetPose(pose); },
        // Supplier of current robot speeds
        [this]
        { return GetState().Speeds; },
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards)
        {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(speeds)
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        []
        {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );

    // Configure simulation if in simulation mode
    if (frc::RobotBase::IsSimulation())
    {
        ConfigureSimulation();
    }
}

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled())
    {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor)
        {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::ConfigureSimulation()
{
    if (!frc::RobotBase::IsSimulation())
        return;

    // 初始化仿真管理器
    simulation::SimulationManager::GetInstance().Initialize();

    // 创建Swerve仿真组件 - 使用硬编码的模块位置
    std::array<frc::Translation2d, 4> modulePositions = {
        frc::Translation2d{11.279527545_in, 10.96456691795_in},  // Front Left
        frc::Translation2d{11.279527545_in, -10.96456691795_in}, // Front Right
        frc::Translation2d{-11.279527545_in, 10.96456691795_in}, // Back Left
        frc::Translation2d{-11.279527545_in, -10.96456691795_in} // Back Right
    };

    frc::SwerveDriveKinematics<4> kinematics{modulePositions};
    m_swerveSimulation = std::make_unique<simulation::SwerveSimulation>(kinematics, modulePositions);

    // 注册仿真更新回调
    simulation::SimulationManager::GetInstance().AddUpdateCallback([this]()
                                                                   {
        if (m_swerveSimulation) {
            m_swerveSimulation->UpdateSimulation();
        } });

    SimulationInit();
}

void CommandSwerveDrivetrain::SimulationInit()
{
    if (!frc::RobotBase::IsSimulation())
        return;

    // Phoenix 6 Swerve模拟会自动处理设备方向和传感器数据
    // 这里可以添加任何额外的模拟初始化代码
    std::cout << "Swerve simulation initialized" << std::endl;
}

void CommandSwerveDrivetrain::SimulationPeriodic()
{
    if (!frc::RobotBase::IsSimulation())
        return;

    units::second_t currentTime = frc::Timer::GetFPGATimestamp();
    units::second_t deltaTime = currentTime - m_lastSimTime;

    if (deltaTime >= kSimLoopPeriod)
    {
        // 使用Phoenix 6内置的Swerve模拟 - 这会正确处理所有的传感器数据和物理模拟
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());

        // 更新我们的仿真组件
        if (m_swerveSimulation)
        {
            // 获取当前模块状态并传递给仿真
            auto moduleStates = GetState().ModuleStates;
            std::array<frc::SwerveModuleState, 4> states;
            for (size_t i = 0; i < 4; ++i)
            {
                states[i] = moduleStates[i];
            }
            m_swerveSimulation->SetModuleStates(states);

            // 同步位置
            m_swerveSimulation->ResetPose(GetState().Pose);
        }

        // 更新仿真管理器
        simulation::SimulationManager::GetInstance().Periodic();

        m_lastSimTime = currentTime;
    }
}

// Unit conversion helper methods for simulation
units::meter_t CommandSwerveDrivetrain::RotationsToMeters(units::turn_t rotations)
{
    // Every radian of rotation, the wheel travels this many meters
    constexpr units::meter_t kWheelRadius = 2_in;
    constexpr auto wheelDistancePerRad = kWheelRadius / 1_rad;
    // Now apply gear ratio to input rotations
    auto gearedRotations = rotations / 6.48; // Drive gear ratio
    // And multiply geared rotations by meters per rotation
    return gearedRotations * wheelDistancePerRad;
}

units::turn_t CommandSwerveDrivetrain::MetersToRotations(units::meter_t meters)
{
    // Every radian of rotation, the wheel travels this many meters
    constexpr units::meter_t kWheelRadius = 2_in;
    constexpr auto wheelDistancePerRad = kWheelRadius / 1_rad;
    // Now get wheel rotations from input meters
    auto wheelRadians = meters / wheelDistancePerRad;
    // And multiply by gear ratio to get rotor rotations
    return wheelRadians * 6.48; // Drive gear ratio
}

units::meters_per_second_t CommandSwerveDrivetrain::RotationsToMetersVel(units::turns_per_second_t rotations)
{
    // Every radian of rotation, the wheel travels this many meters
    constexpr units::meter_t kWheelRadius = 2_in;
    constexpr auto wheelDistancePerRad = kWheelRadius / 1_rad;
    // Now apply gear ratio to input rotations
    auto gearedRotations = rotations / 6.48; // Drive gear ratio
    // And multiply geared rotations by meters per rotation
    return gearedRotations * wheelDistancePerRad;
}

units::turns_per_second_t CommandSwerveDrivetrain::MetersToRotationsVel(units::meters_per_second_t meters)
{
    // Every radian of rotation, the wheel travels this many meters
    constexpr units::meter_t kWheelRadius = 2_in;
    constexpr auto wheelDistancePerRad = kWheelRadius / 1_rad;
    // Now get wheel rotations from input meters
    auto wheelRadians = meters / wheelDistancePerRad;
    // And multiply by gear ratio to get rotor rotations
    return wheelRadians * 6.48; // Drive gear ratio
}