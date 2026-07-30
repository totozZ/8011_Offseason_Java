package frc.robot.health.analyzer.model;

import java.util.Objects;

/** A named interval where {@code /Health/TestSession/Active} was true. */
public record TestSessionInterval(
    String name, long startMicros, long endMicros, int ordinal) {
  public TestSessionInterval {
    name = Objects.requireNonNullElse(name, "Unnamed");
    if (startMicros < 0L || endMicros < startMicros) {
      throw new IllegalArgumentException("invalid test-session interval");
    }
    if (ordinal < 1) {
      throw new IllegalArgumentException("ordinal must be positive");
    }
  }

  public double durationSeconds() {
    return (endMicros - startMicros) / 1_000_000.0;
  }
}
