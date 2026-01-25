// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/GPDetection.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "subsystems/CommandSwerveDrivetrain.h"

namespace {
constexpr const char kLimelightTableName[] = "limelight-back";
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr int kMedianWindowSize = 5;
constexpr double kTriMinAbsTyDeg = 3.0;
constexpr double kTriFullAbsTyDeg = 12.0;
constexpr double kPixMinSizePixels = 12.0;
constexpr double kPixFullSizePixels = 60.0;
constexpr double kMinTargetAreaPercent = 0.1;

constexpr size_t kT2dTargetValid = 0;
constexpr size_t kT2dTargetCount = 1;
constexpr size_t kT2dTxNc = 6;
constexpr size_t kT2dTyNc = 7;
constexpr size_t kT2dTa = 8;
constexpr size_t kT2dLongSidePixels = 12;
constexpr size_t kT2dShortSidePixels = 13;
constexpr size_t kT2dHorizExtentPixels = 14;
constexpr size_t kT2dVertExtentPixels = 15;
constexpr size_t kT2dFieldCount = 17;

constexpr double kCameraHeightMeters = 0.148;
constexpr double kCameraPitchDegrees = 23.1;
constexpr double kAlgaeHeightMeters = 0.413;
constexpr double kLollipopHeightMeters = 0.3048 + (kAlgaeHeightMeters * 0.5);

constexpr double kFx = 748.3768;
constexpr double kFy = 748.0703;
}  // namespace

namespace subsystems {

GPDetection::GPDetection(std::string in_name,
                         CommandSwerveDrivetrain* drivetrain) {
  (void)in_name;
  m_drivetrain = drivetrain;
  m_tablePtr =
      nt::NetworkTableInstance::GetDefault().GetTable(kLimelightTableName);
}

void GPDetection::Periodic() {
  try {
    const std::string objectSeen =
        m_tablePtr->GetString("tdclass", "nothing seen");
    m_valid = (objectSeen == "algae");
    if (!m_valid) {
      m_azimuthAverage = 209;
      m_fieldPoseXSamples.clear();
      m_fieldPoseYSamples.clear();
      m_hasValidFieldPose = false;
      return;
    }

    const auto t2dValues =
        m_tablePtr->GetNumberArray("t2d", std::vector<double>{});
    const bool hasT2d = t2dValues.size() >= kT2dFieldCount;
    const bool t2dHasTarget = hasT2d && t2dValues[kT2dTargetValid] > 0.5 &&
                              t2dValues[kT2dTargetCount] > 0.0;

    if (!t2dHasTarget) {
      m_valid = false;
      m_azimuthAverage = 209;
      m_fieldPoseXSamples.clear();
      m_fieldPoseYSamples.clear();
      m_hasValidFieldPose = false;
      return;
    }

    const double txnc_degrees = t2dValues[kT2dTxNc];
    m_azimuthAverage = -txnc_degrees;

    const double ty_degrees = t2dValues[kT2dTyNc];
    const double targetArea = t2dValues[kT2dTa];
    double widthPixels = 0.0;
    double heightPixels = 0.0;

    const double longSidePixels = t2dValues[kT2dLongSidePixels];
    const double shortSidePixels = t2dValues[kT2dShortSidePixels];
    if (longSidePixels > 0.0 && shortSidePixels > 0.0) {
      widthPixels = longSidePixels;
      heightPixels = shortSidePixels;
    } else {
      const double t2dWidth = t2dValues[kT2dHorizExtentPixels];
      const double t2dHeight = t2dValues[kT2dVertExtentPixels];
      if (t2dWidth > 0.0 && t2dHeight > 0.0) {
        widthPixels = t2dWidth;
        heightPixels = t2dHeight;
      }
    }

    const double distance_pixels =
        CalculateDistancePixelSize(ty_degrees, widthPixels, heightPixels);
    const double distance_tri = CalculateDistanceTriangulation(ty_degrees);
    const double fusedDistance = CalculateFusedDistance(
        ty_degrees, widthPixels, heightPixels, targetArea);

    if (fusedDistance <= 0.0) {
      m_fieldPoseXSamples.clear();
      m_fieldPoseYSamples.clear();
      m_hasValidFieldPose = false;
      return;
    }
    if (m_drivetrain != nullptr) {
      const frc::Pose2d robotPose = m_drivetrain->GetcurrentPose();
      const frc::Pose2d cameraPose = robotPose.TransformBy(m_robotToCam);

      frc::Pose2d object_pose_tri{};
      frc::Pose2d object_pose_pix{};
      frc::Pose2d object_pose_fused{};

      if (distance_tri > 0.0) {
        object_pose_tri = ComputeObjectPoseFromDistance(
            cameraPose, distance_tri, txnc_degrees, nullptr);
      }
      if (distance_pixels > 0.0) {
        object_pose_pix = ComputeObjectPoseFromDistance(
            cameraPose, distance_pixels, txnc_degrees, nullptr);
      }

      object_pose_fused = ComputeObjectPoseFromDistance(
          cameraPose, fusedDistance, txnc_degrees, nullptr);

      const double filtered_x = ApplyMedianFilterPose(
          m_fieldPoseXSamples, object_pose_fused.Translation().X().value());
      const double filtered_y = ApplyMedianFilterPose(
          m_fieldPoseYSamples, object_pose_fused.Translation().Y().value());
      const frc::Pose2d object_pose_filtered{
          frc::Translation2d{units::meter_t{filtered_x},
                             units::meter_t{filtered_y}},
          object_pose_fused.Rotation()};

      m_gamepieceFieldPose = object_pose_filtered;
      m_hasValidFieldPose = true;

      const std::array<double, 3> object_tri_vals{
          object_pose_tri.Translation().X().value(),
          object_pose_tri.Translation().Y().value(),
          object_pose_tri.Rotation().Radians().value()};
      frc::SmartDashboard::PutNumberArray("Object Pose (Triangulation)",
                                          object_tri_vals);

      const std::array<double, 3> object_pix_vals{
          object_pose_pix.Translation().X().value(),
          object_pose_pix.Translation().Y().value(),
          object_pose_pix.Rotation().Radians().value()};
      frc::SmartDashboard::PutNumberArray("Object Pose (Pixels)",
                                          object_pix_vals);

      const std::array<double, 3> object_fused_vals{
          object_pose_fused.Translation().X().value(),
          object_pose_fused.Translation().Y().value(),
          object_pose_fused.Rotation().Radians().value()};
      frc::SmartDashboard::PutNumberArray("Object Pose (Fused)",
                                          object_fused_vals);
      const std::array<double, 3> object_filtered_vals{
          object_pose_filtered.Translation().X().value(),
          object_pose_filtered.Translation().Y().value(),
          object_pose_filtered.Rotation().Radians().value()};
      frc::SmartDashboard::PutNumberArray("Object Pose (Filtered)",
                                          object_filtered_vals);
    } else {
      m_hasValidFieldPose = false;
      m_fieldPoseXSamples.clear();
      m_fieldPoseYSamples.clear();
    }

    frc::SmartDashboard::PutNumber("Distance (Pixels)", distance_pixels);
    frc::SmartDashboard::PutNumber("Distance (Triangulation)", distance_tri);
    frc::SmartDashboard::PutNumber("Distance (Fused)", fusedDistance);

  } catch (const std::exception& e) {
    (void)e;
  }
}

/*
 * Calculates the triangulation distance from the vertical angle.
 */
double GPDetection::CalculateDistanceTriangulation(double ty_degrees) const {
  const double heightDiff = kLollipopHeightMeters - kCameraHeightMeters;
  const double totalAngleRad = (kCameraPitchDegrees + ty_degrees) * kDegToRad;
  const double tanValue = std::tan(totalAngleRad);
  if (std::abs(tanValue) < 1e-6) {
    return -1.0;
  }

  return heightDiff / tanValue;
}

/*
 * Calculates the distance from apparent pixel size.
 */
double GPDetection::CalculateDistancePixelSize(
    double ty_degrees, double objectWidthPixels,
    double objectHeightPixels) const {
  (void)objectWidthPixels;
  const double targetSizeMeters = kAlgaeHeightMeters;
  if (targetSizeMeters <= 0.0) {
    return -1.0;
  }

  double dHeight = -1.0;
  if (objectHeightPixels > 0.0 && kFy > 0.0) {
    dHeight = (targetSizeMeters * kFy) / objectHeightPixels;
  }

  if (dHeight <= 0.0) {
    return -1.0;
  }

  const double tyRad = ty_degrees * kDegToRad;
  const double cosTy = std::cos(tyRad);
  if (std::abs(cosTy) < 1e-6) {
    return -1.0;
  }

  const double thetaRad = (kCameraPitchDegrees + ty_degrees) * kDegToRad;
  return dHeight * std::cos(thetaRad) / cosTy;
}

/*
 * Computes weights used to fuse triangulation and pixel distances.
 */
void GPDetection::ComputeFusionWeights(double ty_degrees,
                                       double objectWidthPixels,
                                       double objectHeightPixels,
                                       double targetArea, double d_tri,
                                       double d_pix, double& triWeight,
                                       double& pixWeight) const {
  triWeight = 0.0;
  pixWeight = 0.0;

  if (d_tri > 0.0) {
    const double absTy = std::abs(ty_degrees);
    if (absTy >= kTriMinAbsTyDeg) {
      const double denom = kTriFullAbsTyDeg - kTriMinAbsTyDeg;
      if (denom > 0.0) {
        triWeight = std::clamp((absTy - kTriMinAbsTyDeg) / denom, 0.0, 1.0);
      }
    }
  }

  const double sizePixels = std::max(objectWidthPixels, objectHeightPixels);
  if (d_pix > 0.0 && sizePixels > 0.0) {
    if (sizePixels >= kPixMinSizePixels) {
      const double denom = kPixFullSizePixels - kPixMinSizePixels;
      if (denom > 0.0) {
        pixWeight =
            std::clamp((sizePixels - kPixMinSizePixels) / denom, 0.0, 1.0);
      }
    }
    if (targetArea > 0.0 && targetArea < kMinTargetAreaPercent) {
      pixWeight = 0.0;
    }
  }
}

double GPDetection::CalculateFusedDistance(double ty_degrees,
                                           double objectWidthPixels,
                                           double objectHeightPixels,
                                           double targetArea) const {
  const double d_tri = CalculateDistanceTriangulation(ty_degrees);
  const double d_pix = CalculateDistancePixelSize(ty_degrees, objectWidthPixels,
                                                  objectHeightPixels);

  double triWeight = 0.0;
  double pixWeight = 0.0;
  ComputeFusionWeights(ty_degrees, objectWidthPixels, objectHeightPixels,
                       targetArea, d_tri, d_pix, triWeight, pixWeight);

  if (triWeight <= 0.0 && pixWeight <= 0.0) {
    if (d_pix > 0.0) {
      return d_pix;
    }
    if (d_tri > 0.0) {
      return d_tri;
    }
    return -1.0;
  }

  if (triWeight <= 0.0) {
    return d_pix;
  }
  if (pixWeight <= 0.0) {
    return d_tri;
  }

  return (triWeight * d_tri + pixWeight * d_pix) / (triWeight + pixWeight);
}

double GPDetection::ApplyMedianFilterPose(std::deque<double>& samples,
                                          double newValue) {
  samples.push_back(newValue);
  if (samples.size() > static_cast<size_t>(kMedianWindowSize)) {
    samples.pop_front();
  }

  if (samples.empty()) {
    return newValue;
  }

  std::vector<double> sorted(samples.begin(), samples.end());
  std::sort(sorted.begin(), sorted.end());
  const size_t mid = sorted.size() / 2;
  if (sorted.size() % 2 == 1) {
    return sorted[mid];
  }
  return 0.5 * (sorted[mid - 1] + sorted[mid]);
}

frc::Pose2d GPDetection::ComputeObjectPoseFromDistance(
    const frc::Pose2d& cameraPose, double distance, double txnc_degrees,
    frc::Pose2d* relativePose) const {
  const double txRad = txnc_degrees * kDegToRad;
  const double relative_left = distance * std::tan(txRad);
  const double relative_forward = -distance;

  const frc::Translation2d rel_trans{units::meter_t{relative_forward},
                                     units::meter_t{relative_left}};
  const frc::Pose2d rel_pose{rel_trans,
                             frc::Rotation2d{units::degree_t{txnc_degrees}}};

  if (relativePose != nullptr) {
    *relativePose = rel_pose;
  }

  const frc::Translation2d field_trans =
      cameraPose.Translation() - rel_trans.RotateBy(cameraPose.Rotation());
  return frc::Pose2d{field_trans, 0_deg};
}

}  // namespace subsystems
