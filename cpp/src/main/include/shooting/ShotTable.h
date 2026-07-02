#pragma once

#include <span>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

namespace shooting {

enum class AllianceSide { kUnknown, kBlue, kRed };

struct ShotSetpoint {
  units::turns_per_second_t flywheel{0.0};
  units::turns_per_second_t upperFeeder{0.0};
  units::degree_t pitch{0.0};
};

struct ShotTableEntry {
  units::meter_t distance{0.0};
  ShotSetpoint setpoint{};
};

class ShotTable {
 public:
  static ShotSetpoint Hub(units::meter_t distance);
  static ShotSetpoint Pass(units::meter_t distance);
  static ShotSetpoint Tower();
  static ShotSetpoint ZeroPitchFallback();
  static bool IsHubRegion(AllianceSide alliance, units::meter_t field_x);

  static constexpr bool FeedAllowed(bool pitch_homed, bool ready,
                                    bool timeout_elapsed) {
    return pitch_homed && (ready || timeout_elapsed);
  }

  static ShotSetpoint Interpolate(std::span<const ShotTableEntry> table,
                                  units::meter_t distance);
};

}  // namespace shooting
