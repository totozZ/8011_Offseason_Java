package frc.robot.health.analyzer.report;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.health.analyzer.analysis.AnalysisResult;
import frc.robot.health.analyzer.analysis.AnalysisSummary;
import frc.robot.health.analyzer.analysis.CurrentStatistics;
import frc.robot.health.analyzer.analysis.SignalStatistics;
import frc.robot.health.analyzer.rules.Anomaly;
import frc.robot.health.analyzer.rules.RuleType;
import frc.robot.health.analyzer.rules.Severity;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ReportExporterTest {
    @TempDir Path temporaryDirectory;

    @Test
    void writesValidJsonCsvAndEscapedHtml() throws Exception {
        AnalysisResult result = new AnalysisResult(
                new AnalysisSummary(
                        5_000_000L,
                        1.0,
                        0.5,
                        Double.NaN,
                        Double.NaN,
                        CurrentStatistics.unavailable("PDH Total Current"),
                        1,
                        0,
                        0,
                        1,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN,
                        Double.NaN),
                List.of(
                        new Anomaly(
                                RuleType.TRACKING_ERROR,
                                Severity.WARNING,
                                6_000_000L,
                                6_250_000L,
                                "Drive",
                                "Left",
                                "reason",
                                "recommendation",
                                Map.of())),
                Map.of(
                        "/Health/Missing",
                        SignalStatistics.unavailable("/Health/Missing")),
                Map.of(),
                Map.of(
                        "Supply Current", "Battery-side <current>",
                        "Stator Current", "Winding current",
                        "PDH Total Current", "Whole robot"));
        Path json = temporaryDirectory.resolve("result.json");
        Path csv = temporaryDirectory.resolve("result.csv");
        Path html = temporaryDirectory.resolve("result.html");
        Files.writeString(json, "old json");
        Files.writeString(csv, "old csv");
        Files.writeString(html, "old html");

        ReportExporter.writeJson(result, json);
        ReportExporter.writeCsv(result, csv);
        ReportExporter.writeHtml(result, html);

        String jsonText = Files.readString(json);
        assertTrue(jsonText.contains("\"startingVoltage\": null"));
        assertTrue(jsonText.contains("\"minimumVoltage\": null"));
        assertTrue(jsonText.contains("\"integralValueSeconds\": null"));
        assertTrue(Files.readString(csv).contains("\"PDH Total Current\""));
        assertTrue(Files.readString(csv).contains("starting_voltage_v,minimum_voltage_v"));
        assertTrue(
                Files.readString(csv)
                        .contains("\"TRACKING_ERROR\",1.000000,0.250000"));
        String htmlText = Files.readString(html);
        assertTrue(htmlText.contains("Supply Current"));
        assertTrue(htmlText.contains("Stator Current"));
        assertTrue(htmlText.contains("PDH Total Current"));
        assertTrue(htmlText.contains("起始电压"));
        assertTrue(htmlText.contains("Battery-side &lt;current&gt;"));
        assertTrue(htmlText.contains("1.000000 s"));
    }

    @Test
    void atomicallyOverwritesExistingTargetAndCleansTemporaryFile() throws Exception {
        Path directory = Files.createDirectory(temporaryDirectory.resolve("overwrite"));
        Path target = directory.resolve("x");
        Files.writeString(target, "old contents");

        ReportExporter.writeAtomically(target, "complete new contents");

        assertEquals("complete new contents", Files.readString(target));
        assertOnlyTargetRemains(directory, target);
    }

    @Test
    void failedTemporaryWritePreservesExistingTargetAndCleansTemporaryFile() throws Exception {
        Path directory = Files.createDirectory(temporaryDirectory.resolve("failure"));
        Path target = directory.resolve("report.json");
        Files.writeString(target, "known-good report");

        IOException exception =
                assertThrows(
                        IOException.class,
                        () ->
                                ReportExporter.writeAtomically(
                                        target,
                                        temporaryPath -> {
                                            Files.writeString(temporaryPath, "partial report");
                                            throw new IOException("injected write failure");
                                        }));

        assertEquals("injected write failure", exception.getMessage());
        assertEquals("known-good report", Files.readString(target));
        assertOnlyTargetRemains(directory, target);
    }

    @Test
    void concurrentWritesToSameTargetPublishOnlyOneCompleteFile() throws Exception {
        Path directory = Files.createDirectory(temporaryDirectory.resolve("concurrent"));
        Path target = directory.resolve("same-name.html");
        Files.writeString(target, "old contents");
        String firstContents = "first:" + "A".repeat(16_384);
        String secondContents = "second:" + "B".repeat(16_384);
        CountDownLatch temporaryFilesReady = new CountDownLatch(2);
        CountDownLatch releaseMoves = new CountDownLatch(1);
        ExecutorService executor = Executors.newFixedThreadPool(2);
        try {
            Future<Void> first =
                    stageConcurrentWrite(
                            executor,
                            target,
                            firstContents,
                            temporaryFilesReady,
                            releaseMoves);
            Future<Void> second =
                    stageConcurrentWrite(
                            executor,
                            target,
                            secondContents,
                            temporaryFilesReady,
                            releaseMoves);

            assertTrue(
                    temporaryFilesReady.await(5, TimeUnit.SECONDS),
                    "both exports should finish their independent temporary files");
            releaseMoves.countDown();
            first.get(5, TimeUnit.SECONDS);
            second.get(5, TimeUnit.SECONDS);
        } finally {
            releaseMoves.countDown();
            executor.shutdownNow();
            assertTrue(executor.awaitTermination(5, TimeUnit.SECONDS));
        }

        String published = Files.readString(target);
        assertTrue(
                published.equals(firstContents) || published.equals(secondContents),
                "the target must contain one complete export");
        assertOnlyTargetRemains(directory, target);
        assertEquals(0, ReportExporter.activeTargetLockCount());
    }

    private static Future<Void> stageConcurrentWrite(
            ExecutorService executor,
            Path target,
            String contents,
            CountDownLatch temporaryFilesReady,
            CountDownLatch releaseMoves) {
        return executor.submit(
                () -> {
                    ReportExporter.writeAtomically(
                            target,
                            temporaryPath -> {
                                Files.writeString(temporaryPath, contents);
                                temporaryFilesReady.countDown();
                                try {
                                    if (!releaseMoves.await(5, TimeUnit.SECONDS)) {
                                        throw new IOException("timed out waiting to publish");
                                    }
                                } catch (InterruptedException exception) {
                                    Thread.currentThread().interrupt();
                                    throw new IOException(
                                            "interrupted while waiting to publish", exception);
                                }
                            });
                    return null;
                });
    }

    private static void assertOnlyTargetRemains(Path directory, Path target) throws IOException {
        try (var files = Files.list(directory)) {
            assertEquals(
                    List.of(target.getFileName().toString()),
                    files.map(path -> path.getFileName().toString()).sorted().toList());
        }
    }
}
