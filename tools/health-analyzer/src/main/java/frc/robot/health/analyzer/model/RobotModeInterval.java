package frc.robot.health.analyzer.model;

import java.util.Objects;

/** A half-open robot-mode interval on the global log timeline. */
public record RobotModeInterval(String mode, long startMicros, long endMicros) {
  public RobotModeInterval {
    Objects.requireNonNull(mode, "mode");
    if (startMicros < 0) {
      throw new IllegalArgumentException("startMicros must be non-negative");
    }
    if (endMicros < startMicros) {
      throw new IllegalArgumentException("endMicros must not precede startMicros");
    }
  }

  public long durationMicros() {
    return endMicros - startMicros;
  }

  public boolean contains(long timestampMicros) {
    return timestampMicros >= startMicros && timestampMicros < endMicros;
  }
}
