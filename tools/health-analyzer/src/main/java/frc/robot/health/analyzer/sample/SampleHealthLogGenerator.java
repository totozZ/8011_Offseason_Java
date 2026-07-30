package frc.robot.health.analyzer.sample;

import frc.robot.health.analyzer.analysis.AnalysisResult;
import frc.robot.health.analyzer.analysis.HealthAnalyzer;
import frc.robot.health.analyzer.io.WpiLogLoader;
import frc.robot.health.analyzer.io.wpilog.PureJavaWpiLogWriter;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.SignalType;
import frc.robot.health.analyzer.rules.HealthRules;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

/**
 * Generates a small, deterministic health WPILOG without loading WPILib JNI.
 *
 * <p>The fixture is intentionally down-sampled to 10 Hz. It contains normal operation followed by
 * voltage sag, brownout, a stalled left drive motor, a disconnected right drive motor, elevated
 * CAN utilization, and idle current draw.
 */
public final class SampleHealthLogGenerator {
  public static final String DEFAULT_FILE_NAME = "robot-health-sample.wpilog";

  private static final long SAMPLE_PERIOD_MICROS = 100_000;
  private static final long END_MICROS = 15_000_000;
  private static final String ROOT = "/Health";
  private static final String DRIVE = ROOT + "/Subsystems/Drive";
  private static final String LEFT = ROOT + "/Motors/Drive/Left";
  private static final String RIGHT = ROOT + "/Motors/Drive/Right";

  private static final List<String> BOOLEAN_PATHS =
      List.of(
          ROOT + "/Robot/Enabled",
          ROOT + "/Robot/BrownedOut",
          ROOT + "/TestSession/Active",
          DRIVE + "/Active",
          LEFT + "/Connected",
          LEFT + "/SupplyCurrentLimited",
          LEFT + "/StatorCurrentLimited",
          RIGHT + "/Connected",
          RIGHT + "/SupplyCurrentLimited",
          RIGHT + "/StatorCurrentLimited");

  private static final List<String> DOUBLE_PATHS =
      List.of(
          ROOT + "/Power/Voltage",
          ROOT + "/Power/TotalCurrent",
          ROOT + "/Robot/CAN/Utilization",
          DRIVE + "/SupplyCurrent",
          DRIVE + "/StatorCurrent",
          DRIVE + "/Reference",
          DRIVE + "/Measured",
          DRIVE + "/ClosedLoopError",
          LEFT + "/SupplyCurrent",
          LEFT + "/StatorCurrent",
          LEFT + "/MotorVoltage",
          LEFT + "/Reference",
          LEFT + "/Velocity",
          LEFT + "/ClosedLoopError",
          LEFT + "/Temperature",
          RIGHT + "/SupplyCurrent",
          RIGHT + "/StatorCurrent",
          RIGHT + "/MotorVoltage",
          RIGHT + "/Reference",
          RIGHT + "/Velocity",
          RIGHT + "/ClosedLoopError",
          RIGHT + "/Temperature");

  private static final List<String> INTEGER_PATHS =
      List.of(
          ROOT + "/Robot/CAN/BusOffCount",
          ROOT + "/Robot/CAN/ReceiveErrors",
          ROOT + "/Robot/CAN/TransmitErrors",
          LEFT + "/Faults",
          RIGHT + "/Faults");

  private static final List<String> STRING_PATHS =
      List.of(
          ROOT + "/Robot/Mode",
          ROOT + "/TestSession/Name",
          ROOT + "/Events/Category",
          ROOT + "/Events/Message",
          DRIVE + "/State",
          DRIVE + "/CurrentCommand",
          DRIVE + "/CurrentSource",
          LEFT + "/ControlMode",
          RIGHT + "/ControlMode");

  private SampleHealthLogGenerator() {}

  /**
   * Writes a sample to the optional first argument, or to {@code
   * build/samples/robot-health-sample.wpilog} when run directly without arguments.
   */
  public static void main(String[] args) throws Exception {
    if (args.length > 1) {
      throw new IllegalArgumentException(
          "Usage: SampleHealthLogGenerator [output-file.wpilog]");
    }
    Path requested =
        args.length == 1
            ? Path.of(args[0])
            : Path.of("build", "samples", DEFAULT_FILE_NAME);
    Path output = generate(requested);

    HealthLog log = new WpiLogLoader().load(output);
    AnalysisResult analysis = HealthAnalyzer.analyze(log, HealthRules.loadDefaults());
    if (log.signalPaths().isEmpty() || analysis.anomalies().isEmpty()) {
      throw new IOException("Generated sample did not pass loader/analyzer self-validation");
    }

    System.out.printf(
        Locale.ROOT,
        "Generated %s (%d bytes, %d signals, %d events, %.1f s, %d default-rule anomalies)%n",
        output,
        Files.size(output),
        log.signalPaths().size(),
        log.events().size(),
        log.durationMicros() / 1_000_000.0,
        analysis.anomalies().size());
  }

  /** Generates the deterministic sample and returns its absolute normalized path. */
  public static Path generate(Path output) throws IOException {
    Objects.requireNonNull(output, "output");
    Path destination = output.toAbsolutePath().normalize();
    Path parent = destination.getParent();
    if (parent != null) {
      Files.createDirectories(parent);
    }

    try (PureJavaWpiLogWriter writer =
        new PureJavaWpiLogWriter(destination, "health-analyzer deterministic sample v1")) {
      Entries entries = new Entries(writer);
      for (long timestamp = 0;
          timestamp <= END_MICROS;
          timestamp += SAMPLE_PERIOD_MICROS) {
        appendFrame(entries, timestamp);
        appendEvent(entries, timestamp);
      }
    }
    return destination;
  }

  private static void appendFrame(Entries entries, long timestamp) throws IOException {
    boolean enabled = timestamp >= 2_000_000 && timestamp < 14_000_000;
    boolean brownedOut = timestamp >= 9_400_000 && timestamp < 9_900_000;
    MotorSample left = leftMotorAt(timestamp);
    MotorSample right = rightMotorAt(timestamp);
    PowerSample power = powerAt(timestamp);

    entries.updateBoolean(ROOT + "/Robot/Enabled", enabled, timestamp);
    entries.updateBoolean(ROOT + "/Robot/BrownedOut", brownedOut, timestamp);
    entries.updateBoolean(ROOT + "/TestSession/Active", enabled, timestamp);
    entries.updateBoolean(DRIVE + "/Active", enabled, timestamp);
    appendMotorBooleans(entries, LEFT, left, timestamp);
    appendMotorBooleans(entries, RIGHT, right, timestamp);

    entries.updateString(ROOT + "/Robot/Mode", modeAt(timestamp), timestamp);
    entries.updateString(
        ROOT + "/TestSession/Name", enabled ? "Sample Drive Health" : "None", timestamp);
    entries.updateString(DRIVE + "/State", driveStateAt(timestamp), timestamp);
    entries.updateString(DRIVE + "/CurrentCommand", commandAt(timestamp), timestamp);
    entries.updateString(DRIVE + "/CurrentSource", "TalonFXSupplyCurrent", timestamp);
    entries.updateString(LEFT + "/ControlMode", left.controlMode(), timestamp);
    entries.updateString(RIGHT + "/ControlMode", right.controlMode(), timestamp);

    entries.appendDouble(ROOT + "/Power/Voltage", power.voltage(), timestamp);
    entries.appendDouble(ROOT + "/Power/TotalCurrent", power.totalCurrent(), timestamp);
    entries.appendDouble(
        ROOT + "/Robot/CAN/Utilization",
        timestamp >= 10_600_000 && timestamp < 11_500_000 ? 0.92 : 0.28,
        timestamp);
    entries.updateInteger(
        ROOT + "/Robot/CAN/BusOffCount", timestamp >= 11_000_000 ? 1L : 0L, timestamp);
    entries.updateInteger(
        ROOT + "/Robot/CAN/ReceiveErrors", timestamp >= 10_900_000 ? 3L : 0L, timestamp);
    entries.updateInteger(
        ROOT + "/Robot/CAN/TransmitErrors", timestamp >= 11_100_000 ? 2L : 0L, timestamp);
    entries.updateInteger(
        LEFT + "/Faults",
        timestamp >= 9_000_000 && timestamp < 10_200_000 ? 0x04L : 0L,
        timestamp);
    entries.updateInteger(
        RIGHT + "/Faults",
        timestamp >= 11_000_000 && timestamp < 12_000_000 ? 0x01L : 0L,
        timestamp);

    double subsystemReference = (left.reference() + right.reference()) / 2.0;
    double subsystemMeasured = (left.velocity() + right.velocity()) / 2.0;
    entries.appendDouble(
        DRIVE + "/SupplyCurrent", left.supplyCurrent() + right.supplyCurrent(), timestamp);
    entries.appendDouble(
        DRIVE + "/StatorCurrent", left.statorCurrent() + right.statorCurrent(), timestamp);
    entries.appendDouble(DRIVE + "/Reference", subsystemReference, timestamp);
    entries.appendDouble(DRIVE + "/Measured", subsystemMeasured, timestamp);
    entries.appendDouble(
        DRIVE + "/ClosedLoopError", subsystemReference - subsystemMeasured, timestamp);

    appendMotorDoubles(entries, LEFT, left, timestamp);
    appendMotorDoubles(entries, RIGHT, right, timestamp);
  }

  private static void appendMotorBooleans(
      Entries entries, String base, MotorSample sample, long timestamp) throws IOException {
    entries.updateBoolean(base + "/Connected", sample.connected(), timestamp);
    entries.updateBoolean(
        base + "/SupplyCurrentLimited", sample.supplyCurrentLimited(), timestamp);
    entries.updateBoolean(
        base + "/StatorCurrentLimited", sample.statorCurrentLimited(), timestamp);
  }

  private static void appendMotorDoubles(
      Entries entries, String base, MotorSample sample, long timestamp) throws IOException {
    entries.appendDouble(base + "/SupplyCurrent", sample.supplyCurrent(), timestamp);
    entries.appendDouble(base + "/StatorCurrent", sample.statorCurrent(), timestamp);
    entries.appendDouble(base + "/MotorVoltage", sample.motorVoltage(), timestamp);
    entries.appendDouble(base + "/Reference", sample.reference(), timestamp);
    entries.appendDouble(base + "/Velocity", sample.velocity(), timestamp);
    entries.appendDouble(
        base + "/ClosedLoopError", sample.reference() - sample.velocity(), timestamp);
    entries.appendDouble(base + "/Temperature", sample.temperature(), timestamp);
  }

  private static void appendEvent(Entries entries, long timestamp) throws IOException {
    switch ((int) (timestamp / SAMPLE_PERIOD_MICROS)) {
      case 0 -> entries.event("Sample", "Deterministic synthetic health log started", timestamp);
      case 20 -> entries.event("TestSessionStart", "Sample Drive Health", timestamp);
      case 80 -> entries.event("Operator", "Applied high-load drive command", timestamp);
      case 90 -> entries.event("Fault", "Injected left drive stall and voltage sag", timestamp);
      case 94 -> entries.event("Brownout", "RobotController reported brownout", timestamp);
      case 110 -> entries.event("MotorFault", "Drive/Right connection lost", timestamp);
      case 120 -> entries.event("MotorFaultCleared", "Drive/Right connection restored", timestamp);
      case 140 -> entries.event("TestSessionStop", "Sample Drive Health", timestamp);
      default -> {
        // No event at this sample.
      }
    }
  }

  private static String modeAt(long timestamp) {
    if (timestamp < 2_000_000 || timestamp >= 14_000_000) {
      return "Disabled";
    }
    return timestamp < 6_000_000 ? "Autonomous" : "Teleop";
  }

  private static String driveStateAt(long timestamp) {
    if (timestamp < 2_000_000 || timestamp >= 14_000_000) {
      return "Idle";
    }
    if (timestamp < 8_000_000) {
      return "Tracking";
    }
    if (timestamp < 10_200_000) {
      return "HighLoad";
    }
    if (timestamp < 12_000_000) {
      return "Fault";
    }
    return "Tracking";
  }

  private static String commandAt(long timestamp) {
    if (timestamp < 2_000_000 || timestamp >= 14_000_000) {
      return "None";
    }
    return timestamp < 6_000_000 ? "DriveTrajectory" : "TeleopDrive";
  }

  private static PowerSample powerAt(long timestamp) {
    if (timestamp < 2_000_000) {
      return new PowerSample(12.6, 5.0);
    }
    if (timestamp < 8_000_000) {
      return new PowerSample(11.5, 78.0);
    }
    if (timestamp < 9_000_000) {
      return new PowerSample(9.2, 215.0);
    }
    if (timestamp < 10_200_000) {
      return new PowerSample(6.4, 285.0);
    }
    if (timestamp < 14_000_000) {
      return new PowerSample(11.1, 92.0);
    }
    return new PowerSample(12.4, 18.0);
  }

  private static MotorSample leftMotorAt(long timestamp) {
    if (timestamp < 2_000_000) {
      return new MotorSample(0.5, 0.5, 0.0, 0.0, 0.0, 30.0, true, false, false);
    }
    if (timestamp < 8_000_000) {
      double temperature = 30.0 + secondsSince(timestamp, 2_000_000) * 0.3;
      return new MotorSample(
          22.0, 36.0, 6.0, 35.0, 34.5, temperature, true, false, false);
    }
    if (timestamp < 9_000_000) {
      double temperature = 33.0 + secondsSince(timestamp, 8_000_000) * 2.0;
      return new MotorSample(
          48.0, 65.0, 8.0, 50.0, 46.0, temperature, true, false, false);
    }
    if (timestamp < 10_200_000) {
      double temperature = 70.0 + secondsSince(timestamp, 9_000_000) * 15.0;
      return new MotorSample(
          60.0, 100.0, 9.0, 55.0, 0.2, temperature, true, true, true);
    }
    if (timestamp < 12_000_000) {
      double temperature = 88.0 - secondsSince(timestamp, 10_200_000) * 4.0;
      return new MotorSample(
          30.0, 45.0, 6.0, 40.0, 39.0, temperature, true, false, false);
    }
    if (timestamp < 14_000_000) {
      double temperature = 75.0 - secondsSince(timestamp, 12_000_000) * 2.0;
      return new MotorSample(
          24.0, 38.0, 5.5, 36.0, 35.5, temperature, true, false, false);
    }
    return new MotorSample(12.0, 14.0, 2.0, 0.0, 0.0, 70.0, true, false, false);
  }

  private static MotorSample rightMotorAt(long timestamp) {
    if (timestamp < 2_000_000) {
      return new MotorSample(0.5, 0.5, 0.0, 0.0, 0.0, 29.0, true, false, false);
    }
    if (timestamp < 8_000_000) {
      double temperature = 29.0 + secondsSince(timestamp, 2_000_000) * 0.25;
      return new MotorSample(
          21.0, 34.0, 6.0, 35.0, 34.0, temperature, true, false, false);
    }
    if (timestamp < 9_000_000) {
      return new MotorSample(38.0, 55.0, 8.0, 50.0, 44.0, 35.0, true, false, false);
    }
    if (timestamp < 10_200_000) {
      return new MotorSample(18.0, 30.0, 7.0, 55.0, 38.0, 45.0, true, false, false);
    }
    if (timestamp < 11_000_000) {
      return new MotorSample(20.0, 32.0, 5.0, 35.0, 34.0, 48.0, true, false, false);
    }
    if (timestamp < 12_000_000) {
      return new MotorSample(0.5, 0.5, 0.0, 40.0, 0.0, 48.0, false, false, false);
    }
    if (timestamp < 14_000_000) {
      return new MotorSample(23.0, 36.0, 5.5, 36.0, 35.0, 47.0, true, false, false);
    }
    return new MotorSample(0.5, 0.5, 0.0, 0.0, 0.0, 45.0, true, false, false);
  }

  private static double secondsSince(long timestamp, long startTimestamp) {
    return (timestamp - startTimestamp) / 1_000_000.0;
  }

  private record PowerSample(double voltage, double totalCurrent) {}

  private record MotorSample(
      double supplyCurrent,
      double statorCurrent,
      double motorVoltage,
      double reference,
      double velocity,
      double temperature,
      boolean connected,
      boolean supplyCurrentLimited,
      boolean statorCurrentLimited) {
    String controlMode() {
      if (!connected) {
        return "DisabledOut";
      }
      return Math.abs(motorVoltage) > 0.0 ? "VoltageOut" : "NeutralOut";
    }
  }

  private static final class Entries {
    private final PureJavaWpiLogWriter writer;
    private final Map<String, Integer> booleans = new LinkedHashMap<>();
    private final Map<String, Integer> doubles = new LinkedHashMap<>();
    private final Map<String, Integer> integers = new LinkedHashMap<>();
    private final Map<String, Integer> strings = new LinkedHashMap<>();
    private final Map<String, Boolean> lastBooleans = new LinkedHashMap<>();
    private final Map<String, Long> lastIntegers = new LinkedHashMap<>();
    private final Map<String, String> lastStrings = new LinkedHashMap<>();

    Entries(PureJavaWpiLogWriter writer) throws IOException {
      this.writer = writer;
      for (String path : BOOLEAN_PATHS) {
        booleans.put(path, writer.start(path, SignalType.BOOLEAN, 0));
      }
      for (String path : DOUBLE_PATHS) {
        doubles.put(path, writer.start(path, SignalType.DOUBLE, 0));
      }
      for (String path : INTEGER_PATHS) {
        integers.put(path, writer.start(path, SignalType.INTEGER, 0));
      }
      for (String path : STRING_PATHS) {
        strings.put(path, writer.start(path, SignalType.STRING, 0));
      }
    }

    void updateBoolean(String path, boolean value, long timestamp) throws IOException {
      Boolean previous = lastBooleans.put(path, value);
      if (previous == null || previous != value) {
        writer.appendBoolean(entry(booleans, path), value, timestamp);
      }
    }

    void appendDouble(String path, double value, long timestamp) throws IOException {
      writer.appendDouble(entry(doubles, path), value, timestamp);
    }

    void updateInteger(String path, long value, long timestamp) throws IOException {
      Long previous = lastIntegers.put(path, value);
      if (previous == null || previous != value) {
        writer.appendInteger(entry(integers, path), value, timestamp);
      }
    }

    void updateString(String path, String value, long timestamp) throws IOException {
      String previous = lastStrings.put(path, value);
      if (!Objects.equals(previous, value)) {
        writer.appendString(entry(strings, path), value, timestamp);
      }
    }

    void event(String category, String message, long timestamp) throws IOException {
      writer.appendString(entry(strings, ROOT + "/Events/Category"), category, timestamp);
      writer.appendString(entry(strings, ROOT + "/Events/Message"), message, timestamp);
    }

    private static int entry(Map<String, Integer> entries, String path) {
      Integer entry = entries.get(path);
      if (entry == null) {
        throw new IllegalArgumentException("Unregistered sample signal: " + path);
      }
      return entry;
    }
  }
}
