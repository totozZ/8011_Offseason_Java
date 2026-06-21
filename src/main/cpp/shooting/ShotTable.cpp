#include "shooting/ShotTable.h"

#include <array>
#include <algorithm>

#include "Constants.h"

using namespace units::literals;

namespace shooting {
namespace {

constexpr units::turns_per_second_t Rpm(double rpm) {
  return units::turns_per_second_t{rpm / 60.0};
}

// Initial values are converted from 1678's 2026 public mechanism-RPM tables.
// These are calibration starting points, not final competition values.
constexpr std::array<ShotTableEntry, 10> kHubTable{{
    {1.700_m, {Rpm(1855.0), Rpm(1425.0), 17.5_deg}},
    {2.000_m, {Rpm(1875.0), Rpm(1439.0), 18.5_deg}},
    {2.215_m, {Rpm(1940.0), Rpm(1450.0), 19.0_deg}},
    {2.495_m, {Rpm(2015.0), Rpm(1500.0), 19.5_deg}},
    {2.888_m, {Rpm(2065.0), Rpm(1550.0), 20.5_deg}},
    {3.222_m, {Rpm(2130.0), Rpm(1650.0), 21.5_deg}},
    {3.570_m, {Rpm(2200.0), Rpm(1750.0), 22.5_deg}},
    {3.830_m, {Rpm(2240.0), Rpm(1800.0), 23.0_deg}},
    {4.150_m, {Rpm(2340.0), Rpm(1950.0), 24.5_deg}},
    {4.750_m, {Rpm(2365.0), Rpm(2150.0), 26.5_deg}},
}};

// Pass pitch remains in 8011's existing coordinate system. 1678's ferry hood
// angle uses a different zero reference and cannot be copied directly.
constexpr std::array<ShotTableEntry, 10> kPassTable{{
    {4.977_m, {Rpm(1800.0), Rpm(1650.0), 30.0_deg}},
    {5.976_m, {Rpm(1850.0), Rpm(1700.0), 30.0_deg}},
    {6.902_m, {Rpm(1900.0), Rpm(1750.0), 30.0_deg}},
    {7.912_m, {Rpm(2050.0), Rpm(1850.0), 30.0_deg}},
    {8.477_m, {Rpm(2100.0), Rpm(1900.0), 30.0_deg}},
    {9.029_m, {Rpm(2200.0), Rpm(2000.0), 30.0_deg}},
    {9.590_m, {Rpm(2400.0), Rpm(2200.0), 30.0_deg}},
    {10.061_m, {Rpm(2550.0), Rpm(2350.0), 30.0_deg}},
    {12.460_m, {Rpm(3150.0), Rpm(1400.0), 30.0_deg}},
    {13.133_m, {Rpm(3500.0), Rpm(1300.0), 30.0_deg}},
}};

}  // namespace

ShotSetpoint ShotTable::Interpolate(std::span<const ShotTableEntry> table,
                                    units::meter_t distance) {
  if (table.empty()) {
    return {};
  }
  if (distance <= table.front().distance) {
    return table.front().setpoint;
  }
  if (distance >= table.back().distance) {
    return table.back().setpoint;
  }

  const auto upper = std::upper_bound(
      table.begin(), table.end(), distance,
      [](units::meter_t value, const ShotTableEntry& entry) {
        return value < entry.distance;
      });
  const auto lower = upper - 1;
  const double ratio =
      ((distance - lower->distance) / (upper->distance - lower->distance))
          .value();

  return {
      lower->setpoint.flywheel +
          (upper->setpoint.flywheel - lower->setpoint.flywheel) * ratio,
      lower->setpoint.upperFeeder +
          (upper->setpoint.upperFeeder - lower->setpoint.upperFeeder) * ratio,
      lower->setpoint.pitch +
          (upper->setpoint.pitch - lower->setpoint.pitch) * ratio,
  };
}

ShotSetpoint ShotTable::Hub(units::meter_t distance) {
  return Interpolate(kHubTable, distance);
}

ShotSetpoint ShotTable::Pass(units::meter_t distance) {
  return Interpolate(kPassTable, distance);
}

ShotSetpoint ShotTable::Tower() { return Hub(3.048_m); }

ShotSetpoint ShotTable::ZeroPitchFallback() {
  return {Rpm(1815.0), Rpm(1400.0), 0.0_deg};
}

bool ShotTable::IsHubRegion(AllianceSide alliance, units::meter_t field_x) {
  switch (alliance) {
    case AllianceSide::kBlue:
      return field_x <= FieldConstants::kHubPassBlueBoundaryX;
    case AllianceSide::kRed:
      return field_x >= FieldConstants::kHubPassRedBoundaryX;
    case AllianceSide::kUnknown:
    default:
      return false;
  }
}

}  // namespace shooting
