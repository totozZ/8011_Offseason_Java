package frc.robot.health.analyzer.app;

import frc.robot.health.analyzer.analysis.AnalysisResult;
import frc.robot.health.analyzer.analysis.HealthAnalyzer;
import frc.robot.health.analyzer.io.WpiLogLoader;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.TestSessionInterval;
import frc.robot.health.analyzer.report.ReportExporter;
import frc.robot.health.analyzer.rules.HealthRules;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import javafx.application.Application;

/**
 * Plain launcher avoids the JDK's special JavaFX main-class handling and also provides a headless
 * report smoke-test path.
 */
public final class HealthAnalyzerLauncher {
  private HealthAnalyzerLauncher() {}

  public static void main(String[] args) {
    if (args.length > 0 && "--analyze".equals(args[0])) {
      int exitCode = runHeadless(args);
      if (exitCode != 0) {
        System.exit(exitCode);
      }
      return;
    }
    Application.launch(HealthAnalyzerApp.class, args);
  }

  private static int runHeadless(String[] args) {
    if (args.length < 2) {
      System.err.println(
          "用法: health-analyzer --analyze <file.wpilog> [--rules <health-rules.json>]"
              + " [--session <last|full|N>] [--export-dir <directory>]");
      return 2;
    }
    try {
      Path input = Path.of(args[1]).toAbsolutePath().normalize();
      Path rulesPath = optionPath(args, "--rules");
      Path exportDirectory = optionPath(args, "--export-dir");
      String session = optionValue(args, "--session");
      HealthRules rules =
          rulesPath == null ? HealthRules.loadDefaults() : HealthRules.load(rulesPath);
      HealthLog selectedLog =
          selectSession(new WpiLogLoader().load(input), session == null ? "last" : session);
      AnalysisResult result =
          HealthAnalyzer.analyze(selectedLog, rules);

      if (exportDirectory != null) {
        Files.createDirectories(exportDirectory);
        String stem = fileStem(input);
        ReportExporter.writeJson(result, exportDirectory.resolve(stem + "-health.json"));
        ReportExporter.writeCsv(result, exportDirectory.resolve(stem + "-health.csv"));
        ReportExporter.writeHtml(result, exportDirectory.resolve(stem + "-health.html"));
      }

      System.out.printf(
          Locale.ROOT,
          "Analyzed %s: duration=%.3fs enabled=%.3fs startVoltage=%.3fV"
              + " minVoltage=%.3fV anomalies=%d%n",
          input,
          result.summary().testDurationSeconds(),
          result.summary().enabledSeconds(),
          result.summary().startingVoltage(),
          result.summary().minimumVoltage(),
          result.summary().anomalyCount());
      return 0;
    } catch (Exception exception) {
      System.err.println("分析失败: " + exception.getMessage());
      exception.printStackTrace(System.err);
      return 1;
    }
  }

  private static Path optionPath(String[] args, String option) {
    String value = optionValue(args, option);
    return value == null ? null : Path.of(value).toAbsolutePath().normalize();
  }

  private static String optionValue(String[] args, String option) {
    for (int index = 2; index < args.length; index++) {
      if (option.equals(args[index])) {
        if (index + 1 >= args.length) {
          throw new IllegalArgumentException(option + " 缺少值");
        }
        return args[index + 1];
      }
    }
    return null;
  }

  static HealthLog selectSession(HealthLog source, String selection) {
    String normalized =
        selection == null ? "last" : selection.trim().toLowerCase(Locale.ROOT);
    if (normalized.equals("full")) {
      return source;
    }
    if (normalized.equals("last")) {
      if (source.testSessionIntervals().isEmpty()) {
        return source;
      }
      TestSessionInterval interval =
          source.testSessionIntervals().get(source.testSessionIntervals().size() - 1);
      return source.slice(interval.startMicros(), interval.endMicros());
    }
    final int ordinal;
    try {
      ordinal = Integer.parseInt(normalized);
    } catch (NumberFormatException exception) {
      throw new IllegalArgumentException(
          "--session 必须是 last、full 或正整数会话编号: " + selection,
          exception);
    }
    TestSessionInterval interval =
        source.testSessionIntervals().stream()
            .filter(candidate -> candidate.ordinal() == ordinal)
            .findFirst()
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        "日志中不存在测试会话 #" + ordinal));
    return source.slice(interval.startMicros(), interval.endMicros());
  }

  static String fileStem(Path path) {
    String name = path.getFileName().toString();
    int dot = name.lastIndexOf('.');
    return dot > 0 ? name.substring(0, dot) : name;
  }
}
