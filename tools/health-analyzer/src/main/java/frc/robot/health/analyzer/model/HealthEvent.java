package frc.robot.health.analyzer.model;

import java.util.Objects;

/** A category/message event reconstructed from the health event signal pair. */
public record HealthEvent(long timestampMicros, String category, String message) {
  public HealthEvent {
    if (timestampMicros < 0) {
      throw new IllegalArgumentException("timestampMicros must be non-negative");
    }
    Objects.requireNonNull(category, "category");
    Objects.requireNonNull(message, "message");
  }
}
