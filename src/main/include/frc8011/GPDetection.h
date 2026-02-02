// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>

#include <deque>
#include <memory>
#include <string>

namespace subsystems {

class CommandSwerveDrivetrain;

class GPDetection : public frc2::SubsystemBase {
 public:
  GPDetection(std::string name, CommandSwerveDrivetrain* drivetrain);

  void Periodic() override;

  bool GetValid() const { return m_valid; }
  const frc::Pose2d& GetFilteredPose() const { return m_gamepieceFieldPose; }
  
  bool HasValidFieldPose() const { return m_hasValidFieldPose; }

 private:
  double CalculateDistanceTriangulation(double ty_degrees) const;
  double CalculateDistancePixelSize(double ty_degrees, double objectWidthPixels,
                                    double objectHeightPixels) const;
  double CalculateFusedDistance(double ty_degrees, double objectWidthPixels,
                                double objectHeightPixels,
                                double targetArea) const;
  double ApplyMedianFilterPose(std::deque<double>& samples, double newValue);
  frc::Pose2d ComputeObjectPoseFromDistance(const frc::Pose2d& cameraPose,
                                            double distance,
                                            double txnc_degrees,
                                            frc::Pose2d* relativePose) const;
  void ComputeFusionWeights(double ty_degrees, double objectWidthPixels,
                            double objectHeightPixels, double targetArea,
                            double d_tri, double d_pix, double& triWeight,
                            double& pixWeight) const;

  frc::Pose2d m_gamepieceFieldPose;
  frc::Transform2d m_robotToCam{frc::Translation2d{-0.32559_m, 0_m},
                                frc::Rotation2d{180_deg}};
  bool m_hasValidFieldPose = false;
  bool m_valid = false;
  double m_azimuthAverage = 209.0;
  std::shared_ptr<nt::NetworkTable> m_tablePtr;
  std::deque<double> m_fieldPoseXSamples;
  std::deque<double> m_fieldPoseYSamples;
  CommandSwerveDrivetrain* m_drivetrain = nullptr;
};

}  // namespace subsystems
