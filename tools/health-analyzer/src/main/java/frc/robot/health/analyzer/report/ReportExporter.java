package frc.robot.health.analyzer.report;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;
import frc.robot.health.analyzer.analysis.AnalysisResult;
import frc.robot.health.analyzer.analysis.CurrentStatistics;
import frc.robot.health.analyzer.analysis.MotorAnalysis;
import frc.robot.health.analyzer.analysis.SubsystemAnalysis;
import frc.robot.health.analyzer.rules.Anomaly;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantLock;

/** Offline JSON, CSV, and self-contained HTML report output. */
public final class ReportExporter {
    private static final Gson GSON = new GsonBuilder()
            .registerTypeAdapter(Double.class, new FiniteDoubleAdapter())
            .registerTypeAdapter(double.class, new FiniteDoubleAdapter())
            .serializeNulls()
            .setPrettyPrinting()
            .create();
    private static final ConcurrentHashMap<Path, TargetLock> TARGET_LOCKS =
            new ConcurrentHashMap<>();

    private ReportExporter() {}

    public static void writeJson(AnalysisResult result, Path path) throws IOException {
        writeAtomically(path, GSON.toJson(result));
    }

    /**
     * Writes subsystem current/energy statistics. Current kind is a required column so supply,
     * stator, and PDH total current are never silently mixed.
     */
    public static void writeCsv(AnalysisResult result, Path path) throws IOException {
        StringBuilder csv = new StringBuilder();
        csv.append(
                "scope,subsystem,motor,current_type,average_amps,raw_peak_amps,"
                        + "rolling_100ms_peak_amps,rolling_500ms_peak_amps,p95_amps,p99_amps,"
                        + "seconds_above_threshold,amp_hours,energy_wh,active_seconds,current_source,"
                        + "starting_voltage_v,minimum_voltage_v,"
                        + "max_temperature_c,max_temperature_rise_c_per_s,"
                        + "mean_abs_reference_error,max_abs_reference_error,"
                        + "high_current_low_velocity_s,mean_imbalance_ratio,max_imbalance_ratio,"
                        + "imbalance_seconds,severity,rule,start_seconds,duration_seconds,"
                        + "reason,recommendation\n");
        appendCurrentRow(
                csv,
                "robot",
                "",
                "",
                result.summary().pdhTotalCurrent(),
                result.summary().consumedWattHours(),
                result.summary().enabledSeconds(),
                "PDH_TOTAL_CURRENT",
                result.summary().startingVoltage(),
                result.summary().minimumVoltage(),
                result.summary().maximumTemperatureC(),
                result.summary().maximumTemperatureRiseCPerSecond(),
                Double.NaN,
                result.summary().maximumAbsoluteReferenceError(),
                result.summary().maximumHighCurrentLowVelocitySeconds(),
                Double.NaN,
                Double.NaN,
                Double.NaN);
        for (SubsystemAnalysis subsystem : result.subsystemStatistics().values()) {
            appendCurrentRow(
                    csv,
                    "subsystem",
                    subsystem.subsystem(),
                    "",
                    subsystem.supplyCurrent(),
                    subsystem.energyWattHours(),
                    subsystem.activeSeconds(),
                    subsystem.supplyCurrentSource(),
                    Double.NaN,
                    Double.NaN,
                    subsystem.maximumTemperatureC(),
                    subsystem.maximumTemperatureRiseCPerSecond(),
                    subsystem.meanAbsoluteReferenceError(),
                    subsystem.maximumAbsoluteReferenceError(),
                    subsystem.maximumHighCurrentLowVelocitySeconds(),
                    subsystem.meanMotorImbalanceRatio(),
                    subsystem.maximumMotorImbalanceRatio(),
                    subsystem.imbalanceSecondsAboveThreshold());
            subsystem.motors().values().forEach(motor -> {
                appendCurrentRow(
                        csv,
                        "motor",
                        subsystem.subsystem(),
                        motor.motor(),
                        motor.supplyCurrent(),
                        Double.NaN,
                        subsystem.activeSeconds(),
                        "TALONFX",
                        Double.NaN,
                        Double.NaN,
                        motor.temperature().maximum(),
                        motor.maximumTemperatureRiseCPerSecond(),
                        motor.meanAbsoluteReferenceError(),
                        motor.maximumAbsoluteReferenceError(),
                        motor.highCurrentLowVelocitySeconds(),
                        Double.NaN,
                        Double.NaN,
                        Double.NaN);
                appendCurrentRow(
                        csv,
                        "motor",
                        subsystem.subsystem(),
                        motor.motor(),
                        motor.statorCurrent(),
                        Double.NaN,
                        subsystem.activeSeconds(),
                        "TALONFX",
                        Double.NaN,
                        Double.NaN,
                        motor.temperature().maximum(),
                        motor.maximumTemperatureRiseCPerSecond(),
                        motor.meanAbsoluteReferenceError(),
                        motor.maximumAbsoluteReferenceError(),
                        motor.highCurrentLowVelocitySeconds(),
                        Double.NaN,
                        Double.NaN,
                        Double.NaN);
            });
        }
        for (Anomaly anomaly : result.anomalies()) {
            appendAnomalyRow(
                    csv,
                    anomaly,
                    result.summary().analysisStartMicros());
        }
        writeAtomically(path, csv.toString());
    }

    public static void writeHtml(AnalysisResult result, Path path) throws IOException {
        StringBuilder html = new StringBuilder("""
                <!doctype html>
                <html lang="zh-CN"><head><meta charset="utf-8">
                <meta name="viewport" content="width=device-width,initial-scale=1">
                <title>机器人健康体检报告</title>
                <style>
                body{font:14px system-ui,sans-serif;max-width:1200px;margin:auto;padding:24px;color:#17202a}
                h1,h2{color:#143d59}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px}
                .card{padding:14px;border:1px solid #ccd6dd;border-radius:8px;background:#f7fafc}
                table{border-collapse:collapse;width:100%;margin:12px 0 24px}th,td{border:1px solid #ccd6dd;padding:7px;text-align:left}
                th{background:#edf3f6}.CRITICAL,.ERROR{color:#a61b1b}.WARNING{color:#8a5a00}
                </style></head><body><h1>机器人健康体检报告</h1><div class="cards">
                """);
        card(html, "测试总时间", seconds(result.summary().testDurationSeconds()));
        card(html, "Enabled 时间", seconds(result.summary().enabledSeconds()));
        card(html, "起始电压", measurement(result.summary().startingVoltage(), " V"));
        card(html, "最低电压", measurement(result.summary().minimumVoltage(), " V"));
        card(
                html,
                "PDH 总电流峰值",
                measurement(result.summary().pdhTotalCurrent().rawPeakAmps(), " A"));
        card(
                html,
                "PDH 100 ms 峰值",
                measurement(
                        result.summary().pdhTotalCurrent().rolling100msPeakAmps(),
                        " A"));
        card(
                html,
                "PDH 500 ms 峰值",
                measurement(
                        result.summary().pdhTotalCurrent().rolling500msPeakAmps(),
                        " A"));
        card(
                html,
                "PDH P95 / P99",
                measurement(result.summary().pdhTotalCurrent().p95Amps(), " A")
                        + " / "
                        + measurement(
                                result.summary().pdhTotalCurrent().p99Amps(),
                                " A"));
        card(html, "消耗电量", measurement(result.summary().consumedAmpHours(), " Ah"));
        card(html, "消耗能量", measurement(result.summary().consumedWattHours(), " Wh"));
        card(
                html,
                "Brownout",
                result.summary().brownoutCount() == null
                        ? "Unavailable"
                        : Integer.toString(result.summary().brownoutCount()));
        card(html, "异常", Integer.toString(result.summary().anomalyCount()));
        card(
                html,
                "最高温度",
                measurement(result.summary().maximumTemperatureC(), " °C"));
        card(
                html,
                "最快温升",
                measurement(
                        result.summary().maximumTemperatureRiseCPerSecond(), " °C/s"));
        card(
                html,
                "最大跟踪误差",
                measurement(result.summary().maximumAbsoluteReferenceError(), ""));
        card(
                html,
                "高流低速最长持续",
                seconds(result.summary().maximumHighCurrentLowVelocitySeconds()));
        card(
                html,
                "最高 CAN 利用率",
                measurement(result.summary().maximumCanUtilization() * 100.0, " %"));
        html.append("</div><h2>电流定义</h2><table><tr><th>名称</th><th>含义</th></tr>");
        for (Map.Entry<String, String> definition : result.currentDefinitions().entrySet()) {
            html.append("<tr><td>")
                    .append(escape(definition.getKey()))
                    .append("</td><td>")
                    .append(escape(definition.getValue()))
                    .append("</td></tr>");
        }
        html.append("</table><h2>子系统</h2><table><tr><th>子系统</th><th>电流来源</th>"
                + "<th>平均 Supply Current</th><th>峰值</th><th>100ms 峰值</th>"
                + "<th>Wh</th><th>Active</th><th>最高温度</th><th>最快温升</th>"
                + "<th>平均/最大误差</th><th>高流低速</th><th>最大不平衡</th></tr>");
        for (SubsystemAnalysis subsystem : result.subsystemStatistics().values()) {
            html.append("<tr><td>").append(escape(subsystem.subsystem())).append("</td><td>")
                    .append(escape(subsystem.supplyCurrentSource())).append("</td><td>")
                    .append(measurement(subsystem.supplyCurrent().averageAmps(), " A")).append("</td><td>")
                    .append(measurement(subsystem.supplyCurrent().rawPeakAmps(), " A")).append("</td><td>")
                    .append(measurement(subsystem.supplyCurrent().rolling100msPeakAmps(), " A")).append("</td><td>")
                    .append(measurement(subsystem.energyWattHours(), " Wh")).append("</td><td>")
                    .append(seconds(subsystem.activeSeconds())).append("</td><td>")
                    .append(measurement(subsystem.maximumTemperatureC(), " °C")).append("</td><td>")
                    .append(measurement(subsystem.maximumTemperatureRiseCPerSecond(), " °C/s")).append("</td><td>")
                    .append(measurement(subsystem.meanAbsoluteReferenceError(), ""))
                    .append(" / ")
                    .append(measurement(subsystem.maximumAbsoluteReferenceError(), ""))
                    .append("</td><td>")
                    .append(seconds(subsystem.maximumHighCurrentLowVelocitySeconds())).append("</td><td>")
                    .append(measurement(subsystem.maximumMotorImbalanceRatio() * 100.0, " %"))
                    .append("</td></tr>");
        }
        html.append("</table><h2>电机</h2><table><tr><th>对象</th>"
                + "<th>平均/峰值 Supply</th><th>平均/峰值 Stator</th>"
                + "<th>最高温度</th><th>最快温升</th><th>平均/最大误差</th>"
                + "<th>高流低速</th></tr>");
        for (SubsystemAnalysis subsystem : result.subsystemStatistics().values()) {
            for (MotorAnalysis motor : subsystem.motors().values()) {
                html.append("<tr><td>")
                        .append(escape(motor.subsystem() + "/" + motor.motor()))
                        .append("</td><td>")
                        .append(measurement(motor.supplyCurrent().averageAmps(), " A"))
                        .append(" / ")
                        .append(measurement(motor.supplyCurrent().rawPeakAmps(), " A"))
                        .append("</td><td>")
                        .append(measurement(motor.statorCurrent().averageAmps(), " A"))
                        .append(" / ")
                        .append(measurement(motor.statorCurrent().rawPeakAmps(), " A"))
                        .append("</td><td>")
                        .append(measurement(motor.temperature().maximum(), " °C"))
                        .append("</td><td>")
                        .append(measurement(motor.maximumTemperatureRiseCPerSecond(), " °C/s"))
                        .append("</td><td>")
                        .append(measurement(motor.meanAbsoluteReferenceError(), ""))
                        .append(" / ")
                        .append(measurement(motor.maximumAbsoluteReferenceError(), ""))
                        .append("</td><td>")
                        .append(seconds(motor.highCurrentLowVelocitySeconds()))
                        .append("</td></tr>");
            }
        }
        html.append("</table><h2>异常</h2><table><tr><th>严重程度</th><th>规则</th>"
                + "<th>开始</th><th>持续</th><th>对象</th><th>原因</th><th>建议</th></tr>");
        for (Anomaly anomaly : result.anomalies()) {
            html.append("<tr><td class=\"").append(anomaly.severity()).append("\">")
                    .append(anomaly.severity()).append("</td><td>").append(anomaly.rule())
                    .append("</td><td>")
                    .append(
                            seconds(
                                    (anomaly.startMicros()
                                                    - result.summary()
                                                            .analysisStartMicros())
                                            / 1_000_000.0))
                    .append("</td><td>").append(seconds(anomaly.durationSeconds()))
                    .append("</td><td>").append(escape(objectName(anomaly)))
                    .append("</td><td>").append(escape(anomaly.reason()))
                    .append("</td><td>").append(escape(anomaly.recommendation()))
                    .append("</td></tr>");
        }
        html.append("</table></body></html>");
        writeAtomically(path, html.toString());
    }

    private static void appendCurrentRow(
            StringBuilder csv,
            String scope,
            String subsystem,
            String motor,
            CurrentStatistics current,
            double energyWh,
            double activeSeconds,
            String source,
            double startingVoltage,
            double minimumVoltage,
            double maximumTemperature,
            double maximumTemperatureRise,
            double meanReferenceError,
            double maximumReferenceError,
            double highCurrentLowVelocitySeconds,
            double meanImbalance,
            double maximumImbalance,
            double imbalanceSeconds) {
        csv.append(csv(scope)).append(',').append(csv(subsystem)).append(',').append(csv(motor))
                .append(',').append(csv(current.currentKind())).append(',')
                .append(number(current.averageAmps())).append(',')
                .append(number(current.rawPeakAmps())).append(',')
                .append(number(current.rolling100msPeakAmps())).append(',')
                .append(number(current.rolling500msPeakAmps())).append(',')
                .append(number(current.p95Amps())).append(',')
                .append(number(current.p99Amps())).append(',')
                .append(number(current.secondsAboveThreshold())).append(',')
                .append(number(current.ampHours())).append(',')
                .append(number(energyWh)).append(',')
                .append(number(activeSeconds)).append(',')
                .append(csv(source)).append(',')
                .append(number(startingVoltage)).append(',')
                .append(number(minimumVoltage)).append(',')
                .append(number(maximumTemperature)).append(',')
                .append(number(maximumTemperatureRise)).append(',')
                .append(number(meanReferenceError)).append(',')
                .append(number(maximumReferenceError)).append(',')
                .append(number(highCurrentLowVelocitySeconds)).append(',')
                .append(number(meanImbalance)).append(',')
                .append(number(maximumImbalance)).append(',')
                .append(number(imbalanceSeconds))
                .append(",,,,,,\n");
    }

    private static void appendAnomalyRow(
            StringBuilder csv, Anomaly anomaly, long analysisStartMicros) {
        csv.append(csv("anomaly")).append(',')
                .append(csv(anomaly.subsystem())).append(',')
                .append(csv(anomaly.motor()));
        for (int column = 4; column <= 26; column++) {
            csv.append(',');
        }
        csv.append(csv(anomaly.severity().toString())).append(',')
                .append(csv(anomaly.rule().toString())).append(',')
                .append(
                        number(
                                (anomaly.startMicros()
                                                - analysisStartMicros)
                                        / 1_000_000.0))
                .append(',')
                .append(number(anomaly.durationSeconds())).append(',')
                .append(csv(anomaly.reason())).append(',')
                .append(csv(anomaly.recommendation())).append('\n');
    }

    private static void card(StringBuilder html, String label, String value) {
        html.append("<div class=\"card\"><strong>").append(escape(label)).append("</strong><br>")
                .append(escape(value)).append("</div>");
    }

    private static String objectName(Anomaly anomaly) {
        if (!anomaly.motor().isBlank()) {
            return anomaly.subsystem() + "/" + anomaly.motor();
        }
        return anomaly.subsystem().isBlank() ? "Robot" : anomaly.subsystem();
    }

    private static String seconds(double value) {
        return measurement(value, " s");
    }

    private static String measurement(double value, String unit) {
        return Double.isFinite(value) ? number(value) + unit : "Unavailable";
    }

    private static String number(double value) {
        return Double.isFinite(value) ? String.format(Locale.ROOT, "%.6f", value) : "";
    }

    private static String csv(String value) {
        String safe = value == null ? "" : value;
        return "\"" + safe.replace("\"", "\"\"") + "\"";
    }

    private static String escape(String value) {
        if (value == null) {
            return "";
        }
        return value.replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
                .replace("\"", "&quot;")
                .replace("'", "&#39;");
    }

    static void writeAtomically(Path path, String contents) throws IOException {
        Objects.requireNonNull(contents, "contents");
        writeAtomically(
                path,
                temporaryPath ->
                        Files.writeString(temporaryPath, contents, StandardCharsets.UTF_8));
    }

    static void writeAtomically(Path path, TemporaryFileWriter writer) throws IOException {
        Objects.requireNonNull(path, "path");
        Objects.requireNonNull(writer, "writer");
        Path target = path.toAbsolutePath().normalize();
        Path parent = target.getParent();
        if (parent == null || target.getFileName() == null) {
            throw new IOException("Report output must name a file: " + target);
        }
        Files.createDirectories(parent);
        parent = parent.toRealPath();
        target = parent.resolve(target.getFileName()).normalize();

        // A unique file in the target directory makes concurrent exports independent and keeps
        // the final move on the same filesystem, which is required for an atomic replacement.
        Path temporaryPath = Files.createTempFile(parent, ".health-report-", ".tmp");
        try {
            writer.write(temporaryPath);
            publishTemporaryFile(temporaryPath, target);
        } catch (IOException | RuntimeException | Error failure) {
            try {
                Files.deleteIfExists(temporaryPath);
            } catch (IOException | RuntimeException cleanupFailure) {
                failure.addSuppressed(cleanupFailure);
            }
            throw failure;
        }
    }

    private static void publishTemporaryFile(Path temporaryPath, Path target) throws IOException {
        TargetLock targetLock = acquireTargetLock(target);
        try {
            try {
                Files.move(
                        temporaryPath,
                        target,
                        StandardCopyOption.ATOMIC_MOVE,
                        StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException exception) {
                // REPLACE_EXISTING does not truncate the old target before the completed temp file
                // is ready. This is the safest available fallback on filesystems without atomic
                // rename support.
                Files.move(temporaryPath, target, StandardCopyOption.REPLACE_EXISTING);
            }
        } finally {
            releaseTargetLock(target, targetLock);
        }
    }

    private static TargetLock acquireTargetLock(Path target) {
        TargetLock targetLock =
                TARGET_LOCKS.compute(
                        target,
                        (ignored, existing) -> {
                            TargetLock selected = existing == null ? new TargetLock() : existing;
                            ++selected.references;
                            return selected;
                        });
        targetLock.lock.lock();
        return targetLock;
    }

    private static void releaseTargetLock(Path target, TargetLock targetLock) {
        targetLock.lock.unlock();
        TARGET_LOCKS.compute(
                target,
                (ignored, existing) -> {
                    if (existing != targetLock) {
                        throw new IllegalStateException("Mismatched report target lock: " + target);
                    }
                    --targetLock.references;
                    return targetLock.references == 0 ? null : targetLock;
                });
    }

    static int activeTargetLockCount() {
        return TARGET_LOCKS.size();
    }

    @FunctionalInterface
    interface TemporaryFileWriter {
        void write(Path temporaryPath) throws IOException;
    }

    private static final class TargetLock {
        private final ReentrantLock lock = new ReentrantLock();
        private int references;
    }

    private static final class FiniteDoubleAdapter extends TypeAdapter<Double> {
        @Override
        public void write(JsonWriter out, Double value) throws IOException {
            if (value == null || !Double.isFinite(value)) {
                out.nullValue();
            } else {
                out.value(value);
            }
        }

        @Override
        public Double read(JsonReader in) throws IOException {
            if (in.peek() == com.google.gson.stream.JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return in.nextDouble();
        }
    }
}
