package frc.robot.health.analyzer.model;

import java.util.Objects;

/**
 * A timestamped value from a WPILOG signal.
 *
 * @param timestampMicros timestamp in the WPILOG microsecond time base
 * @param value decoded signal value
 * @param <T> signal value type
 */
public record Sample<T>(long timestampMicros, T value) {
  public Sample {
    if (timestampMicros < 0) {
      throw new IllegalArgumentException("timestampMicros must be non-negative");
    }
    Objects.requireNonNull(value, "value");
  }
}
