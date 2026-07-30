package frc.robot.health.analyzer.ui;

import java.util.Objects;

/** Marker drawn on the shared health timeline. */
public record TimelineMarker(
    long timestampMicros, Kind kind, String title, String detail) {
  public enum Kind {
    EVENT,
    WARNING,
    ERROR,
    CRITICAL
  }

  public TimelineMarker {
    if (timestampMicros < 0L) {
      throw new IllegalArgumentException("timestampMicros must be non-negative");
    }
    kind = Objects.requireNonNull(kind, "kind");
    title = Objects.requireNonNullElse(title, "");
    detail = Objects.requireNonNullElse(detail, "");
  }
}
