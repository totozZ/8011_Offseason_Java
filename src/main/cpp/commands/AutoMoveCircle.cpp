#include "commands/AutoMoveCircle.h"
#include <cmath>
#include <algorithm>

AutoMoveCircle::AutoMoveCircle(CommandSwerveDrivetrain* drivetrain, 
                               double centerX, double centerY, double radius, 
                               double targetAngleDeg, bool isCCW, 
                               double targetVel, bool stopAtEnd, 
                               double targetHeadingDeg, bool faceTravelDir,
                               bool invertA, bool invertD, double angleOffset)
    : m_drivetrain(drivetrain),
      m_Cx(centerX), m_Cy(centerY), m_radius(radius),
      m_targetAngleDeg(targetAngleDeg), m_isCCW(isCCW),
      m_targetVel(targetVel), m_stopAtEnd(stopAtEnd), 
      m_targetHeadingDeg(targetHeadingDeg), m_faceTravelDir(faceTravelDir),
      m_invertA(invertA), m_invertD(invertD), angleOff(angleOffset)
{
    driveClosed.WithHeadingPID(8, 0, 0.1)
               .WithDeadband(units::meters_per_second_t{0.05})
               .WithRotationalDeadband(units::radians_per_second_t{0.1})
               .WithDriveRequestType(swerve::DriveRequestType::Velocity)
               .WithSteerRequestType(swerve::SteerRequestType::Position)
               .WithMaxAbsRotationalRate(units::radians_per_second_t{3.14*1.5});

    AddRequirements({m_drivetrain});
}

void AutoMoveCircle::Initialize() {
    // 1. 镜像反转逻辑
    if(m_invertD) {
        m_Cy = 8.07 - m_Cy;
        m_targetAngleDeg = -m_targetAngleDeg;
        m_targetHeadingDeg = -m_targetHeadingDeg;
        m_isCCW = !m_isCCW; 
    }
    if(m_invertA) {
        m_Cx = 16.54 - m_Cx;
        m_targetAngleDeg = 180.0 - m_targetAngleDeg;
        m_targetHeadingDeg = 180.0 - m_targetHeadingDeg;
        m_isCCW = !m_isCCW; 
    }

    m_direction = m_isCCW ? 1 : -1;

    // 2. 获取初始角度并计算总行程
    frc::Translation2d centerTrans{units::meter_t{m_Cx}, units::meter_t{m_Cy}};
    frc::Translation2d currentTrans = m_drivetrain->GetState().Pose.Translation();
    
    m_lastAngleFromCenter = (currentTrans - centerTrans).Angle();
    m_accumulatedDegrees = 0.0;

    double startDeg = m_lastAngleFromCenter.Degrees().value();
    double diff = m_targetAngleDeg - startDeg;
    
    if (m_isCCW) {
        while (diff < 0) diff += 360.0;
        while (diff >= 360.0) diff -= 360.0;
    } else {
        diff = -diff;
        while (diff < 0) diff += 360.0;
        while (diff >= 360.0) diff -= 360.0;
    }
    m_totalDegreesNeeded = diff;
    
    // 3. 提前算出物理终点的绝对坐标 (用于 IsFinished 的位置判断)
    double targetRad = m_targetAngleDeg * M_PI / 180.0;
    m_targetPoint = centerTrans + frc::Translation2d{
        units::meter_t{m_radius * std::cos(targetRad)},
        units::meter_t{m_radius * std::sin(targetRad)}
    };
    
    m_distPid.Reset();
}

void AutoMoveCircle::Execute() {
    frc::Translation2d centerTrans{units::meter_t{m_Cx}, units::meter_t{m_Cy}};
    frc::Translation2d currentTrans = m_drivetrain->GetState().Pose.Translation();
    frc::Rotation2d currentAngleFromCenter = (currentTrans - centerTrans).Angle();

    // 1. 角度积分
    double deltaDeg = (currentAngleFromCenter - m_lastAngleFromCenter).Degrees().value();
    m_accumulatedDegrees += deltaDeg * m_direction; 
    m_lastAngleFromCenter = currentAngleFromCenter;

    double thetaRad = currentAngleFromCenter.Radians().value();

    // 2. PID 纠偏 (Radial)
    frc::Translation2d idealPoint = centerTrans + frc::Translation2d{
        units::meter_t{m_radius * std::cos(thetaRad)}, 
        units::meter_t{m_radius * std::sin(thetaRad)}
    };

    double v_radial_x = m_radialPidX.Calculate(currentTrans.X().value(), idealPoint.X().value());
    double v_radial_y = m_radialPidY.Calculate(currentTrans.Y().value(), idealPoint.Y().value());

    // 3. 理论距离与 PID 减速 (Tangential)
    double totalArcLen = m_totalDegreesNeeded * M_PI / 180.0 * m_radius;
    double currentArcLen = m_accumulatedDegrees * M_PI / 180.0 * m_radius;
    double base_v_tangent = m_targetVel;

    if (m_stopAtEnd) {
        base_v_tangent = m_distPid.Calculate(currentArcLen, totalArcLen);
    }
    // 确保基础目标速度不超限
    base_v_tangent = std::clamp(base_v_tangent, -m_targetVel, m_targetVel);

    // 4. 纯切线向量 (基础速度分配)
    double tx = -std::sin(thetaRad);
    double ty =  std::cos(thetaRad);
    double v_tangent_x = tx * m_direction * base_v_tangent;
    double v_tangent_y = ty * m_direction * base_v_tangent;

    // 5. 决定车头朝向 (Heading)
    frc::Rotation2d currentHeading;
    if (m_faceTravelDir) {
        // 直接面向切线方向，WPILib 的 Rotation2d(x,y) 会自动求 atan2
        double an = atan2(ty * m_direction, tx * m_direction);
        currentHeading = frc::Rotation2d{units::radian_t{an + angleOff * m_direction}};
    } else {
        currentHeading = frc::Rotation2d{units::degree_t{m_targetHeadingDeg}};
    }

    // ==========================================
    // 6. 最终速度合成与 Pythagorean 重缩放 (Rescale)
    // ==========================================
    double suppX = v_tangent_x + v_radial_x;
    double suppY = v_tangent_y + v_radial_y;

    // 计算 X 和 Y 的勾股定理模长 (当前合成的总速度大小)
    double current_mag = std::hypot(suppX, suppY);

    // 强制 Rescale：只保留方向，把总速度缩放回我们期望的 base_v_tangent
    // (加一个 1e-6 的极小值判断，防止在完全静止时发生除以 0 的错误)
    if (current_mag > 1e-6) {
        suppX = (suppX / current_mag) * base_v_tangent;
        suppY = (suppY / current_mag) * base_v_tangent;
    } else {
        suppX = 0.0;
        suppY = 0.0;
    }

    // 7. 联盟视角反转 (Alliance Flip)
    auto alliance = frc::DriverStation::GetAlliance();
    bool isRed = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
    if (isRed) {
        suppX = -suppX;
        suppY = -suppY;
        currentHeading = currentHeading + frc::Rotation2d{units::degree_t{180.0}};
    }

    // 8. 发送给底盘
    m_drivetrain->SetControl(driveClosed
        .WithVelocityX(units::meters_per_second_t{suppX})
        .WithVelocityY(units::meters_per_second_t{suppY})
        .WithTargetDirection(currentHeading)
    );
}
// void AutoMoveCircle::Execute() {
//     frc::Translation2d centerTrans{units::meter_t{m_Cx}, units::meter_t{m_Cy}};
//     frc::Translation2d currentTrans = m_drivetrain->GetState().Pose.Translation();
//     frc::Rotation2d currentAngleFromCenter = (currentTrans - centerTrans).Angle();

//     // 1. 角度积分
//     double deltaDeg = (currentAngleFromCenter - m_lastAngleFromCenter).Degrees().value();
//     m_accumulatedDegrees += deltaDeg * m_direction; 
//     m_lastAngleFromCenter = currentAngleFromCenter;

//     double thetaRad = currentAngleFromCenter.Radians().value();

//     // 2. PID 纠偏 (Radial) - 保留你的原逻辑
//     frc::Translation2d idealPoint = centerTrans + frc::Translation2d{
//         units::meter_t{m_radius * std::cos(thetaRad)}, 
//         units::meter_t{m_radius * std::sin(thetaRad)}
//     };

//     double v_radial_x = m_radialPidX.Calculate(currentTrans.X().value(), idealPoint.X().value());
//     double v_radial_y = m_radialPidY.Calculate(currentTrans.Y().value(), idealPoint.Y().value());

//     // 3. 理论距离与 PID 减速 (Tangential)
//     double totalArcLen = m_totalDegreesNeeded * M_PI / 180.0 * m_radius;
//     double currentArcLen = m_accumulatedDegrees * M_PI / 180.0 * m_radius;
//     double base_v_tangent = m_targetVel;

//     if (m_stopAtEnd) {
//         base_v_tangent = m_distPid.Calculate(currentArcLen, totalArcLen);
//     }
//     base_v_tangent = std::clamp(base_v_tangent, -m_targetVel, m_targetVel);

//     // ==========================================
//     // 4. 改为：前瞻虚拟目标点法 (Lookahead Point)
//     // ==========================================
//     // 计算当前角度往前 10 度的位置 (根据顺逆时针决定正负)
//     double lookaheadRad = thetaRad + (10.0 * M_PI / 180.0) * m_direction;
    
//     // 算出该虚拟点在场地上的绝对坐标
//     frc::Translation2d virtualTarget{
//         units::meter_t{m_Cx + m_radius * std::cos(lookaheadRad)},
//         units::meter_t{m_Cy + m_radius * std::sin(lookaheadRad)}
//     };

//     // 计算从当前位置【指向】虚拟点的向量
//     double dx = virtualTarget.X().value() - currentTrans.X().value();
//     double dy = virtualTarget.Y().value() - currentTrans.Y().value();
//     double dist = std::hypot(dx, dy);

//     // 归一化为纯方向向量 (此时它天然已经包含了 m_direction 的顺逆时针信息)
//     double travel_dir_x = dx / dist;
//     double travel_dir_y = dy / dist;

//     // 计算主行驶速度分量
//     double v_tangent_x = travel_dir_x * base_v_tangent;
//     double v_tangent_y = travel_dir_y * base_v_tangent;

//     // ==========================================
//     // 5. 决定车头朝向 (Heading) - 完美保留
//     // ==========================================
//     frc::Rotation2d currentHeading;
//     if (m_faceTravelDir) {
//         // 直接面向我们刚算出的行驶向量，保留你的 angleOff 补偿逻辑
//         double an = std::atan2(travel_dir_y, travel_dir_x);
//         currentHeading = frc::Rotation2d{units::radian_t{an + angleOff * m_direction}};
//     } else {
//         currentHeading = frc::Rotation2d{units::degree_t{m_targetHeadingDeg}};
//     }

//     // 6. 最终速度合成 (完美保留 Radial 纠偏叠加)
//     double suppX = v_tangent_x + v_radial_x;
//     double suppY = v_tangent_y + v_radial_y;

//     // 7. 红方反转处理
//     auto alliance = frc::DriverStation::GetAlliance();
//     bool isRed = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
//     if (isRed) {
//         suppX = -suppX;
//         suppY = -suppY;
//         currentHeading = currentHeading + frc::Rotation2d{units::degree_t{180.0}};
//     }

//     // 8. 发送给底盘
//     m_drivetrain->SetControl(driveClosed
//         .WithVelocityX(units::meters_per_second_t{suppX})
//         .WithVelocityY(units::meters_per_second_t{suppY})
//         .WithTargetDirection(currentHeading)
//     );
// }
bool AutoMoveCircle::IsFinished() {
    frc::Translation2d currentTrans = m_drivetrain->GetState().Pose.Translation();
    double distToTarget = currentTrans.Distance(m_targetPoint).value();

    // 结束条件 1: 位置到达。距离小于 15cm，并且已经走完了至少一半的路程 (防 360度整圆一开机秒退的 Bug)
    bool isPositionReached = (distToTarget < 0.15) && (m_accumulatedDegrees >= m_totalDegreesNeeded * 0.5);
    
    // 结束条件 2: 角度到达。作为一个防弹兜底条件
    bool isAngleReached = m_accumulatedDegrees >= m_totalDegreesNeeded;

    return isPositionReached || isAngleReached;
}

void AutoMoveCircle::End(bool interrupted) {
    // 结束时保持最后算出来的车头朝向
    if (m_stopAtEnd || interrupted) {
        m_drivetrain->SetControl(m_drivetrain->Idle);
    } else {
        auto currentSpeeds = m_drivetrain->GetState().Speeds;
        m_drivetrain->SetControl(m_drivetrain->m_safeCoastRequest
            .WithVelocityX(currentSpeeds.vx)
            .WithVelocityY(currentSpeeds.vy)
            .WithRotationalRate(currentSpeeds.omega) // 保持旋转惯量
        );
    }
}