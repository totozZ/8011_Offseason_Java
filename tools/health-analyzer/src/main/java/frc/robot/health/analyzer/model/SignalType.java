package frc.robot.health.analyzer.model;

import java.util.Arrays;
import java.util.Optional;

/** Scalar signal types supported by the health analyzer. */
public enum SignalType {
  BOOLEAN("boolean"),
  DOUBLE("double"),
  INTEGER("int64"),
  STRING("string");

  private final String wpiType;

  SignalType(String wpiType) {
    this.wpiType = wpiType;
  }

  /** Returns the type string stored in a WPILOG start record. */
  public String wpiType() {
    return wpiType;
  }

  /** Resolves an exact WPILOG scalar type name. */
  public static Optional<SignalType> fromWpiType(String wpiType) {
    return Arrays.stream(values()).filter(type -> type.wpiType.equals(wpiType)).findFirst();
  }
}
