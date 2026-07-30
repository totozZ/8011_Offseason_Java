package frc.robot.health.analyzer.ui;

import frc.robot.health.analyzer.analysis.SeriesMath;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.RobotModeInterval;
import frc.robot.health.analyzer.model.Sample;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;

/** Read-only value-at-cursor view for robot, subsystem, and motor context. */
public final class SnapshotPane extends VBox {
  private static final List<String> SUBSYSTEM_NUMBERS =
      List.of("SupplyCurrent", "StatorCurrent", "Reference", "Measured", "ClosedLoopError");
  private static final List<String> MOTOR_NUMBERS =
      List.of(
          "SupplyCurrent",
          "StatorCurrent",
          "MotorVoltage",
          "Velocity",
          "Reference",
          "Temperature",
          "ClosedLoopError",
          "Faults");

  private final Label timestampLabel = new Label("游标快照");
  private final TextArea details = new TextArea();
  private HealthLog log;
  private List<String> selectedSignals = List.of();
  private long currentTimestampMicros;

  public SnapshotPane() {
    setSpacing(8.0);
    setPadding(new Insets(10.0));
    setPrefWidth(360.0);
    setMinWidth(270.0);
    getStyleClass().add("side-panel");
    timestampLabel.getStyleClass().add("panel-title");
    details.setEditable(false);
    details.setWrapText(false);
    details.getStyleClass().add("snapshot-text");
    details.setText("打开日志后，在时间线上移动鼠标查看该时刻的状态。");
    VBox.setVgrow(details, Priority.ALWAYS);
    getChildren().addAll(timestampLabel, details);
  }

  public void setLog(HealthLog value) {
    log = value;
    if (value == null) {
      timestampLabel.setText("游标快照");
      details.setText("打开日志后，在时间线上移动鼠标查看该时刻的状态。");
    } else {
      currentTimestampMicros = value.startMicros();
      update(value.startMicros());
    }
  }

  public void setSelectedSignals(List<String> paths) {
    selectedSignals = paths == null ? List.of() : List.copyOf(paths);
    if (log != null) {
      update(currentTimestampMicros);
    }
  }

  public void update(long timestampMicros) {
    if (log == null) {
      return;
    }
    long timestamp = Math.max(log.startMicros(), Math.min(log.endMicros(), timestampMicros));
    currentTimestampMicros = timestamp;
    double relativeSeconds = (timestamp - log.startMicros()) / 1_000_000.0;
    timestampLabel.setText(
        String.format(Locale.ROOT, "游标快照  %,.3f s", relativeSeconds));

    StringBuilder text = new StringBuilder(4096);
    section(text, "机器人");
    value(text, "Mode", modeAt(timestamp));
    value(text, "Enabled", latestBoolean("/Health/Robot/Enabled", timestamp));
    value(text, "BrownedOut", latestBoolean("/Health/Robot/BrownedOut", timestamp));
    value(text, "Session", latestString("/Health/TestSession/Name", timestamp));
    number(text, "Voltage", "/Health/Power/Voltage", timestamp);
    value(
        text,
        "Power Distribution",
        powerDistributionStatus(timestamp));
    number(text, "PDH Total Current", "/Health/Power/TotalCurrent", timestamp);
    number(text, "Total Power", "/Health/Power/TotalPower", timestamp);
    number(text, "Loop Time", "/Health/Robot/LoopTimeMs", timestamp);
    number(text, "CAN Utilization", "/Health/Robot/CAN/Utilization", timestamp);
    number(text, "CAN BusOff", "/Health/Robot/CAN/BusOffCount", timestamp);
    number(text, "CAN RX Errors", "/Health/Robot/CAN/ReceiveErrors", timestamp);
    number(text, "CAN TX Errors", "/Health/Robot/CAN/TransmitErrors", timestamp);

    if (!selectedSignals.isEmpty()) {
      section(text, "已选曲线");
      for (String path : selectedSignals) {
        text.append(shortPath(path))
            .append(" = ")
            .append(numericDisplay(log, path, timestamp))
            .append('\n');
      }
    }

    for (String subsystem : subsystemNames()) {
      section(text, "子系统 / " + subsystem);
      value(
          text,
          "Active",
          latestBoolean("/Health/Subsystems/" + subsystem + "/Active", timestamp));
      value(
          text,
          "State",
          latestString("/Health/Subsystems/" + subsystem + "/State", timestamp));
      value(
          text,
          "Command",
          latestString("/Health/Subsystems/" + subsystem + "/CurrentCommand", timestamp));
      value(
          text,
          "Current Source",
          latestString("/Health/Subsystems/" + subsystem + "/CurrentSource", timestamp));
      for (String suffix : SUBSYSTEM_NUMBERS) {
        number(
            text,
            suffix,
            "/Health/Subsystems/" + subsystem + "/" + suffix,
            timestamp);
      }

      for (String motor : motorNames(subsystem)) {
        text.append("  电机 ").append(motor).append('\n');
        for (String suffix : MOTOR_NUMBERS) {
          String path = "/Health/Motors/" + subsystem + "/" + motor + "/" + suffix;
          if (!log.doubleSeries(path).isEmpty()) {
            String display =
                suffix.equals("Faults")
                    ? numericFaultDisplay(log, path, timestamp)
                    : numericDisplay(log, path, timestamp);
            text.append("    ").append(suffix).append(": ").append(display).append('\n');
          }
        }
        Optional<Sample<Boolean>> connected =
            log.latestBoolean(
                "/Health/Motors/" + subsystem + "/" + motor + "/Connected", timestamp);
        connected.ifPresent(
            sample ->
                text.append("    Connected: ").append(sample.value()).append('\n'));
        Optional<Sample<String>> controlMode =
            log.latestString(
                "/Health/Motors/" + subsystem + "/" + motor + "/ControlMode", timestamp);
        controlMode.ifPresent(
            sample ->
                text.append("    ControlMode: ").append(sample.value()).append('\n'));
      }
    }

    details.setText(text.toString());
    details.positionCaret(0);
  }

  private String modeAt(long timestampMicros) {
    for (RobotModeInterval interval : log.modeIntervals()) {
      if (interval.contains(timestampMicros)
          || (timestampMicros == log.endMicros()
              && interval.endMicros() == timestampMicros)) {
        return interval.mode();
      }
    }
    return latestString("/Health/Robot/Mode", timestampMicros);
  }

  private Set<String> subsystemNames() {
    TreeSet<String> names = new TreeSet<>();
    collectNames(log.signalPaths(), "/Health/Subsystems/", names);
    collectNames(log.signalPaths(), "/Health/Motors/", names);
    return names;
  }

  private Set<String> motorNames(String subsystem) {
    TreeSet<String> names = new TreeSet<>();
    collectNames(log.signalPaths(), "/Health/Motors/" + subsystem + "/", names);
    return names;
  }

  private static void collectNames(
      Set<String> paths, String prefix, Set<String> destination) {
    for (String path : paths) {
      if (!path.startsWith(prefix)) {
        continue;
      }
      String remainder = path.substring(prefix.length());
      int slash = remainder.indexOf('/');
      if (slash > 0) {
        destination.add(remainder.substring(0, slash));
      }
    }
  }

  private void number(StringBuilder text, String label, String path, long timestampMicros) {
    text.append(label)
        .append(": ")
        .append(numericDisplay(log, path, timestampMicros))
        .append('\n');
  }

  private static void value(StringBuilder text, String label, Object value) {
    String display = value == null || value.toString().isBlank() ? "Unavailable" : value.toString();
    text.append(label).append(": ").append(display).append('\n');
  }

  private String latestString(String path, long timestampMicros) {
    return log.latestString(path, timestampMicros).map(Sample::value).orElse("Unavailable");
  }

  private String latestBoolean(String path, long timestampMicros) {
    return log.latestBoolean(path, timestampMicros)
        .map(sample -> Boolean.toString(sample.value()))
        .orElse("Unavailable");
  }

  static String numericDisplay(HealthLog log, String path, long timestampMicros) {
    return numericValue(log, path, timestampMicros)
        .map(SnapshotPane::format)
        .orElseGet(
            () ->
                isStale(log, path, timestampMicros)
                    ? "Unavailable (stale)"
                    : "Unavailable");
  }

  private static String numericFaultDisplay(
      HealthLog log, String path, long timestampMicros) {
    return numericValue(log, path, timestampMicros)
        .map(value -> "0x%016X".formatted(Math.round(value)))
        .orElseGet(
            () ->
                isStale(log, path, timestampMicros)
                    ? "Unavailable (stale)"
                    : "Unavailable");
  }

  static Optional<Double> numericValue(
      HealthLog log, String path, long timestampMicros) {
    List<Sample<Double>> samples = log.doubleSeries(path);
    if (samples.isEmpty()) {
      return Optional.empty();
    }
    if (HealthLog.isStepNumericPath(path)) {
      return log.latestDouble(path, timestampMicros).map(Sample::value);
    }
    double interpolated = SeriesMath.valueAt(samples, timestampMicros);
    if (Double.isFinite(interpolated)) {
      return Optional.of(interpolated);
    }
    if (!SeriesMath.hasFreshSampleAt(samples, timestampMicros)) {
      return Optional.empty();
    }
    return log.latestDouble(path, timestampMicros).map(Sample::value);
  }

  private String powerDistributionStatus(long timestampMicros) {
    Optional<Sample<Boolean>> enabled =
        log.latestBoolean("/Health/Power/DistributionEnabled", timestampMicros);
    if (enabled.isEmpty()) {
      return "Unavailable";
    }
    if (!enabled.get().value()) {
      return "未配置（总电流、功率和能量不可用）";
    }
    String type = latestString("/Health/Power/DistributionType", timestampMicros);
    String module =
        numericDisplay(log, "/Health/Power/DistributionModule", timestampMicros);
    return type + " / CAN " + module;
  }

  private static boolean isStale(
      HealthLog log, String path, long timestampMicros) {
    if (HealthLog.isStepNumericPath(path)) {
      return false;
    }
    Optional<Sample<Double>> latest = log.latestDouble(path, timestampMicros);
    return latest.isPresent()
        && timestampMicros - latest.get().timestampMicros()
            > SeriesMath.MAX_CONTINUOUS_GAP_MICROS;
  }

  private static void section(StringBuilder text, String title) {
    if (!text.isEmpty()) {
      text.append('\n');
    }
    text.append("── ").append(title).append(" ──\n");
  }

  private static String shortPath(String path) {
    String prefix = "/Health/";
    return path.startsWith(prefix) ? path.substring(prefix.length()) : path;
  }

  private static String format(double value) {
    if (!Double.isFinite(value)) {
      return "Unavailable";
    }
    double absolute = Math.abs(value);
    if (absolute >= 100_000.0 || (absolute > 0.0 && absolute < 0.001)) {
      return String.format(Locale.ROOT, "%.4e", value);
    }
    return String.format(Locale.ROOT, "%,.4f", value);
  }
}
