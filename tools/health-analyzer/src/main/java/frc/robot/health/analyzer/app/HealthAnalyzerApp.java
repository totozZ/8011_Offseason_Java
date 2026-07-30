package frc.robot.health.analyzer.app;

import frc.robot.health.analyzer.analysis.AnalysisResult;
import frc.robot.health.analyzer.analysis.AnalysisSummary;
import frc.robot.health.analyzer.analysis.CurrentStatistics;
import frc.robot.health.analyzer.analysis.HealthAnalyzer;
import frc.robot.health.analyzer.io.WpiLogLoader;
import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.TestSessionInterval;
import frc.robot.health.analyzer.report.ReportExporter;
import frc.robot.health.analyzer.rules.Anomaly;
import frc.robot.health.analyzer.rules.HealthRules;
import frc.robot.health.analyzer.ui.SignalSelectorPane;
import frc.robot.health.analyzer.ui.SnapshotPane;
import frc.robot.health.analyzer.ui.TimelineChartPane;
import frc.robot.health.analyzer.ui.TimelineMarker;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.OptionalLong;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.property.SimpleStringProperty;
import javafx.collections.FXCollections;
import javafx.concurrent.Task;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.geometry.Rectangle2D;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.SplitPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.Tooltip;
import javafx.scene.input.Dragboard;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.Region;
import javafx.scene.layout.TilePane;
import javafx.scene.layout.VBox;
import javafx.stage.FileChooser;
import javafx.stage.Screen;
import javafx.stage.Stage;

/** Offline JavaFX application for WPILOG health analysis and time-correlated diagnosis. */
public final class HealthAnalyzerApp extends Application {
  private static final double DEFAULT_WINDOW_WIDTH = 1540.0;
  private static final double DEFAULT_WINDOW_HEIGHT = 940.0;
  private static final double MIN_WINDOW_WIDTH = 1120.0;
  private static final double MIN_WINDOW_HEIGHT = 720.0;
  private static final double SCREEN_EDGE_MARGIN = 32.0;

  private final WpiLogLoader loader = new WpiLogLoader();
  private final SignalSelectorPane selector = new SignalSelectorPane();
  private final TimelineChartPane timeline = new TimelineChartPane();
  private final SnapshotPane snapshot = new SnapshotPane();
  private final TableView<Anomaly> anomalyTable = new TableView<>();
  private final Label fileLabel = new Label("尚未打开 WPILOG");
  private final Label rulesLabel = new Label("规则：内置 health-rules.json");
  private final Label statusLabel = new Label("可将 .wpilog 文件拖入窗口");
  private final ProgressIndicator progress = new ProgressIndicator();
  private final ComboBox<SessionChoice> sessionSelector = new ComboBox<>();
  private final Button exportJson = new Button("导出 JSON");
  private final Button exportCsv = new Button("导出 CSV");
  private final Button exportHtml = new Button("导出 HTML");
  private final SummaryCards summaryCards = new SummaryCards();
  private final ExportCoordinator exportCoordinator = new ExportCoordinator();

  private Stage stage;
  private HealthRules rules;
  private Path rulesPath;
  private Path sourcePath;
  private Path selectedPath;
  private Path openPath;
  private HealthLog sourceLog;
  private HealthLog selectedLog;
  private HealthLog currentLog;
  private AnalysisResult currentResult;
  // JavaFX Tasks can finish out of order; only the newest generation may update UI state.
  private Task<?> activeLoadTask;
  private Task<?> activeAnalysisTask;
  private long loadRequestGeneration;
  private long analysisRequestGeneration;
  private boolean exportDataAvailable;
  private boolean updatingSessionSelector;
  private boolean smokeMode;

  @Override
  public void start(Stage primaryStage) {
    stage = primaryStage;
    smokeMode = getParameters().getRaw().contains("--smoke-ui");
    try {
      rules = HealthRules.loadDefaults();
    } catch (IOException exception) {
      rules = new HealthRules();
      Platform.runLater(
          () -> showError("无法读取内置规则", exception));
    }

    BorderPane root = new BorderPane();
    root.getStyleClass().add("app-root");
    root.setTop(buildTop());
    root.setCenter(buildWorkspace());
    root.setBottom(buildAnomalyPanel());
    configureDragAndDrop(root);
    configureExports();

    Scene scene = new Scene(root);
    String stylesheet =
        Objects.requireNonNull(
                getClass().getResource("/ui/health-analyzer.css"),
                "Missing /ui/health-analyzer.css")
            .toExternalForm();
    scene.getStylesheets().add(stylesheet);
    primaryStage.setTitle("FRC 8011 机器人健康分析器");
    primaryStage.setScene(scene);
    fitStageToScreen(primaryStage, Screen.getPrimary().getVisualBounds());
    primaryStage.show();

    List<String> arguments = getParameters().getRaw();
    arguments.stream()
        .map(Path::of)
        .filter(path -> path.toString().toLowerCase(Locale.ROOT).endsWith(".wpilog"))
        .findFirst()
        .ifPresent(this::loadLog);
  }

  private static void fitStageToScreen(Stage target, Rectangle2D visualBounds) {
    double availableWidth = Math.max(1.0, visualBounds.getWidth() - SCREEN_EDGE_MARGIN);
    double availableHeight = Math.max(1.0, visualBounds.getHeight() - SCREEN_EDGE_MARGIN);
    double windowWidth = Math.min(DEFAULT_WINDOW_WIDTH, availableWidth);
    double windowHeight = Math.min(DEFAULT_WINDOW_HEIGHT, availableHeight);

    target.setMinWidth(Math.min(MIN_WINDOW_WIDTH, windowWidth));
    target.setMinHeight(Math.min(MIN_WINDOW_HEIGHT, windowHeight));
    target.setWidth(windowWidth);
    target.setHeight(windowHeight);
    target.setX(visualBounds.getMinX() + (visualBounds.getWidth() - windowWidth) / 2.0);
    target.setY(visualBounds.getMinY() + (visualBounds.getHeight() - windowHeight) / 2.0);
  }

  private VBox buildTop() {
    Button openButton = new Button("打开 WPILOG");
    openButton.getStyleClass().add("primary-button");
    openButton.setOnAction(ignored -> chooseLog());
    Button rulesButton = new Button("加载规则");
    rulesButton.setOnAction(ignored -> chooseRules());
    sessionSelector.setPromptText("测试会话");
    sessionSelector.setPrefWidth(255.0);
    sessionSelector.setDisable(true);
    sessionSelector
        .valueProperty()
        .addListener(
            (ignored, previous, selected) -> {
              if (!updatingSessionSelector
                  && selected != null
                  && sourceLog != null
                  && sourcePath != null) {
                analyzeLoadedLog(sourcePath, selected.select(sourceLog));
              }
            });

    progress.setPrefSize(18.0, 18.0);
    progress.setVisible(false);
    progress.setManaged(false);

    fileLabel.getStyleClass().add("file-label");
    rulesLabel.getStyleClass().add("muted-label");
    statusLabel.getStyleClass().add("muted-label");

    HBox toolbar =
        new HBox(
            9.0,
            openButton,
            rulesButton,
            new Label("分析范围"),
            sessionSelector,
            separator(),
            exportJson,
            exportCsv,
            exportHtml,
            separator(),
            progress,
            statusLabel);
    toolbar.setAlignment(Pos.CENTER_LEFT);
    toolbar.setPadding(new Insets(10.0, 14.0, 5.0, 14.0));
    HBox.setHgrow(statusLabel, Priority.ALWAYS);

    HBox identity = new HBox(18.0, fileLabel, rulesLabel);
    identity.setPadding(new Insets(0.0, 14.0, 7.0, 14.0));
    identity.setAlignment(Pos.CENTER_LEFT);

    setExportsDisabled(true);
    VBox top = new VBox(toolbar, identity, summaryCards.view());
    top.getStyleClass().add("top-panel");
    return top;
  }

  private SplitPane buildWorkspace() {
    selector.setOnSelectionChanged(
        paths -> {
          timeline.setSelectedSignals(paths);
          snapshot.setSelectedSignals(paths);
        });
    timeline.setOnCursorChanged(snapshot::update);

    ScrollPane timelineScroll = new ScrollPane(timeline);
    timelineScroll.setFitToWidth(true);
    timelineScroll.setPannable(false);
    timelineScroll.getStyleClass().add("timeline-scroll");
    timeline.setMinWidth(620.0);

    SplitPane workspace = new SplitPane(selector, timelineScroll, snapshot);
    workspace.setOrientation(Orientation.HORIZONTAL);
    workspace.setDividerPositions(0.18, 0.77);
    return workspace;
  }

  private VBox buildAnomalyPanel() {
    configureAnomalyTable();
    Label title = new Label("异常列表");
    title.getStyleClass().add("panel-title");
    Label hint = new Label("单击异常可跳转到时间线位置");
    hint.getStyleClass().add("muted-label");
    HBox header = new HBox(10.0, title, hint);
    header.setAlignment(Pos.CENTER_LEFT);
    VBox panel = new VBox(6.0, header, anomalyTable);
    panel.setPadding(new Insets(8.0, 12.0, 10.0, 12.0));
    panel.setPrefHeight(240.0);
    VBox.setVgrow(anomalyTable, Priority.ALWAYS);
    panel.getStyleClass().add("anomaly-panel");
    return panel;
  }

  private void configureAnomalyTable() {
    anomalyTable.setPlaceholder(new Label("没有异常，或尚未分析日志"));
    anomalyTable.setColumnResizePolicy(TableView.CONSTRAINED_RESIZE_POLICY);
    anomalyTable
        .getColumns()
        .addAll(
            List.of(
            column("严重程度", 92.0, anomaly -> anomaly.severity().toString()),
            column("开始", 95.0, this::relativeStart),
            column(
                "持续",
                92.0,
                anomaly -> formatSeconds(anomaly.durationSeconds())),
            column("对象", 160.0, HealthAnalyzerApp::objectName),
            column("触发原因", 360.0, Anomaly::reason),
            column("建议检查", 420.0, Anomaly::recommendation)));
    anomalyTable
        .getSelectionModel()
        .selectedItemProperty()
        .addListener(
            (ignored, previous, anomaly) -> {
              if (anomaly != null) {
                timeline.jumpTo(anomaly.startMicros());
              }
            });
    anomalyTable.setRowFactory(
        ignored -> {
          javafx.scene.control.TableRow<Anomaly> row = new javafx.scene.control.TableRow<>();
          row.itemProperty()
              .addListener(
                  (property, oldItem, item) -> {
                    row.getStyleClass().removeAll(
                        "severity-info",
                        "severity-warning",
                        "severity-error",
                        "severity-critical");
                    if (item != null) {
                      row.getStyleClass()
                          .add("severity-" + item.severity().name().toLowerCase(Locale.ROOT));
                      row.setTooltip(
                          new Tooltip(
                              item.reason() + System.lineSeparator() + item.recommendation()));
                    } else {
                      row.setTooltip(null);
                    }
                  });
          return row;
        });
  }

  private TableColumn<Anomaly, String> column(
      String title, double preferredWidth, java.util.function.Function<Anomaly, String> value) {
    TableColumn<Anomaly, String> column = new TableColumn<>(title);
    column.setPrefWidth(preferredWidth);
    column.setCellValueFactory(
        features -> new SimpleStringProperty(value.apply(features.getValue())));
    return column;
  }

  private void chooseLog() {
    FileChooser chooser = new FileChooser();
    chooser.setTitle("打开机器人健康 WPILOG");
    chooser.getExtensionFilters()
        .add(new FileChooser.ExtensionFilter("WPILOG 日志", "*.wpilog"));
    Path initialPath = sourcePath == null ? openPath : sourcePath;
    if (initialPath != null && initialPath.getParent() != null) {
      chooser.setInitialDirectory(initialPath.getParent().toFile());
    }
    File selectedFile = chooser.showOpenDialog(stage);
    if (selectedFile != null) {
      loadLog(selectedFile.toPath());
    }
  }

  private void chooseRules() {
    FileChooser chooser = new FileChooser();
    chooser.setTitle("加载健康规则");
    chooser.getExtensionFilters()
        .add(new FileChooser.ExtensionFilter("JSON 规则", "*.json"));
    File selectedFile = chooser.showOpenDialog(stage);
    if (selectedFile == null) {
      return;
    }
    try {
      HealthRules loaded = HealthRules.load(selectedFile.toPath());
      rules = loaded;
      rulesPath = selectedFile.toPath().toAbsolutePath().normalize();
      rulesLabel.setText("规则：" + rulesPath.getFileName());
      if (activeLoadTask == null && selectedLog != null && selectedPath != null) {
        analyzeLoadedLog(selectedPath, selectedLog);
      }
    } catch (Exception exception) {
      showError("无法加载规则文件", exception);
    }
  }

  private void loadLog(Path path) {
    Path normalized = path.toAbsolutePath().normalize();
    long requestGeneration = ++loadRequestGeneration;
    ++analysisRequestGeneration;
    exportCoordinator.invalidate();
    cancel(activeLoadTask);
    cancel(activeAnalysisTask);
    activeAnalysisTask = null;
    sessionSelector.setDisable(true);
    setExportsDisabled(true);
    setBusy(true, "正在读取 " + normalized.getFileName() + " …");
    Task<HealthLog> task =
        new Task<>() {
          @Override
          protected HealthLog call() throws Exception {
            return loader.load(normalized);
          }
        };
    activeLoadTask = task;
    task.setOnSucceeded(
        ignored -> {
          if (requestGeneration != loadRequestGeneration) {
            return;
          }
          activeLoadTask = null;
          HealthLog loaded = task.getValue();
          if (loaded.signalPaths().isEmpty()) {
            setBusy(false, "日志有效，但没有 /Health/ 标量信号");
            sessionSelector.setDisable(sourceLog == null);
            showInformation(
                "没有健康数据",
                "该 WPILOG 可以读取，但不包含 /Health/ 路径。请确认机器人健康日志已启用。");
            reanalyzeSelectedLog();
            return;
          }
          selectDefaultSessionAndAnalyze(normalized, loaded);
        });
    task.setOnFailed(
        ignored -> {
          if (requestGeneration != loadRequestGeneration) {
            return;
          }
          activeLoadTask = null;
          setBusy(false, "读取失败");
          sessionSelector.setDisable(sourceLog == null);
          showError("无法读取 WPILOG", task.getException());
          reanalyzeSelectedLog();
        });
    startBackground(task, "health-log-loader");
  }

  private void analyzeLoadedLog(Path path, HealthLog log) {
    long requestGeneration = ++analysisRequestGeneration;
    exportCoordinator.invalidate();
    cancel(activeAnalysisTask);
    selectedPath = path;
    selectedLog = log;
    setExportsDisabled(true);
    setBusy(true, "正在计算指标与规则 …");
    HealthRules selectedRules = rules;
    Task<AnalysisResult> task =
        new Task<>() {
          @Override
          protected AnalysisResult call() {
            return HealthAnalyzer.analyze(log, selectedRules);
          }
        };
    activeAnalysisTask = task;
    task.setOnSucceeded(
        ignored -> {
          if (requestGeneration != analysisRequestGeneration) {
            return;
          }
          activeAnalysisTask = null;
          openPath = path;
          currentLog = log;
          currentResult = task.getValue();
          applyAnalysis();
          setBusy(
              false,
              "分析完成：%d 个信号，%d 个异常"
                  .formatted(log.signalPaths().size(), currentResult.anomalies().size()));
        });
    task.setOnFailed(
        ignored -> {
          if (requestGeneration != analysisRequestGeneration) {
            return;
          }
          activeAnalysisTask = null;
          setBusy(false, "分析失败");
          setExportsDisabled(true);
          fileLabel.setText("分析失败；图表保留上次成功结果（不可导出）");
          showError("日志分析失败", task.getException());
        });
    startBackground(task, "health-rule-analyzer");
  }

  private void selectDefaultSessionAndAnalyze(Path path, HealthLog log) {
    sourceLog = log;
    sourcePath = path;
    List<SessionChoice> choices = new java.util.ArrayList<>();
    choices.add(
        new SessionChoice(
            "完整日志 (%.3f s)".formatted(log.durationMicros() / 1_000_000.0),
            log.startMicros(),
            log.endMicros(),
            true));
    for (TestSessionInterval interval : log.testSessionIntervals()) {
      choices.add(
          new SessionChoice(
              "#%d %s (%.3f s)"
                  .formatted(
                      interval.ordinal(), interval.name(), interval.durationSeconds()),
              interval.startMicros(),
              interval.endMicros(),
              false));
    }
    SessionChoice selected = choices.get(choices.size() - 1);
    updatingSessionSelector = true;
    try {
      sessionSelector.setItems(FXCollections.observableArrayList(choices));
      sessionSelector.getSelectionModel().select(selected);
      sessionSelector.setDisable(false);
    } finally {
      updatingSessionSelector = false;
    }
    analyzeLoadedLog(path, selected.select(log));
  }

  private void applyAnalysis() {
    fileLabel.setText(openPath.getFileName() + "  —  " + openPath.getParent());
    summaryCards.update(currentResult.summary(), currentLog);
    selector.setLog(currentLog);
    timeline.setLog(currentLog);
    timeline.setSelectedSignals(selector.selectedSignals());
    snapshot.setLog(currentLog);
    snapshot.setSelectedSignals(selector.selectedSignals());
    timeline.setMarkers(currentResult.anomalies().stream().map(this::marker).toList());
    anomalyTable.setItems(FXCollections.observableArrayList(currentResult.anomalies()));
    setExportsDisabled(false);
    if (smokeMode) {
      runSmokeInteractionAndExit();
    }
  }

  private void runSmokeInteractionAndExit() {
    long midpoint = currentLog.startMicros() + currentLog.durationMicros() / 2L;
    timeline.viewport().zoom(0.5, midpoint);
    timeline.viewport().panFraction(0.1);
    if (!currentResult.anomalies().isEmpty()) {
      timeline.jumpTo(currentResult.anomalies().get(0).startMicros());
    }
    timeline.redraw();
    Rectangle2D visualBounds = Screen.getPrimary().getVisualBounds();
    if (!isWithinVisualBounds(stage, visualBounds)) {
      throw new IllegalStateException(
          "Window extends outside the visual screen bounds: stage=%s,%s %.0fx%.0f; screen=%s"
              .formatted(
                  stage.getX(),
                  stage.getY(),
                  stage.getWidth(),
                  stage.getHeight(),
                  visualBounds));
    }
    System.out.printf(
        Locale.ROOT,
        "JavaFX smoke OK: %d signals, window %.0fx%.0f, zoomed span %.3fs, cursor %.3fs,"
            + " %d anomalies%n",
        currentLog.signalPaths().size(),
        stage.getWidth(),
        stage.getHeight(),
        timeline.viewport().visibleDurationMicros() / 1_000_000.0,
        (timeline.viewport().cursorMicros() - currentLog.startMicros()) / 1_000_000.0,
        currentResult.anomalies().size());
    Platform.runLater(Platform::exit);
  }

  private static boolean isWithinVisualBounds(Stage target, Rectangle2D visualBounds) {
    double tolerance = 1.0;
    return target.getX() >= visualBounds.getMinX() - tolerance
        && target.getY() >= visualBounds.getMinY() - tolerance
        && target.getX() + target.getWidth() <= visualBounds.getMaxX() + tolerance
        && target.getY() + target.getHeight() <= visualBounds.getMaxY() + tolerance;
  }

  private TimelineMarker marker(Anomaly anomaly) {
    TimelineMarker.Kind kind =
        switch (anomaly.severity()) {
          case CRITICAL -> TimelineMarker.Kind.CRITICAL;
          case ERROR -> TimelineMarker.Kind.ERROR;
          case WARNING -> TimelineMarker.Kind.WARNING;
          case INFO -> TimelineMarker.Kind.EVENT;
        };
    return new TimelineMarker(
        anomaly.startMicros(), kind, anomaly.rule().toString(), anomaly.reason());
  }

  private void configureDragAndDrop(BorderPane root) {
    root.setOnDragOver(
        event -> {
          Dragboard board = event.getDragboard();
          if (board.hasFiles()
              && board.getFiles().stream()
                  .anyMatch(
                      file ->
                          file.getName().toLowerCase(Locale.ROOT).endsWith(".wpilog"))) {
            event.acceptTransferModes(TransferMode.COPY);
          }
          event.consume();
        });
    root.setOnDragDropped(
        event -> {
          Dragboard board = event.getDragboard();
          File file =
              board.getFiles().stream()
                  .filter(
                      candidate ->
                          candidate.getName().toLowerCase(Locale.ROOT).endsWith(".wpilog"))
                  .findFirst()
                  .orElse(null);
          event.setDropCompleted(file != null);
          if (file != null) {
            loadLog(file.toPath());
          }
          event.consume();
        });
  }

  private void configureExports() {
    exportJson.setOnAction(
        ignored -> chooseExport("JSON 分析结果", "json", ReportExporter::writeJson));
    exportCsv.setOnAction(
        ignored -> chooseExport("CSV 统计结果", "csv", ReportExporter::writeCsv));
    exportHtml.setOnAction(
        ignored -> chooseExport("HTML 体检报告", "html", ReportExporter::writeHtml));
  }

  private void chooseExport(
      String title, String extension, BiConsumerWithIOException<AnalysisResult, Path> exporter) {
    if (!exportDataAvailable
        || exportCoordinator.isInProgress()
        || currentResult == null
        || openPath == null) {
      return;
    }
    FileChooser chooser = new FileChooser();
    chooser.setTitle(title);
    chooser.getExtensionFilters()
        .add(
            new FileChooser.ExtensionFilter(
                extension.toUpperCase(Locale.ROOT), "*." + extension));
    chooser.setInitialFileName(
        HealthAnalyzerLauncher.fileStem(openPath) + "-health." + extension);
    if (openPath.getParent() != null) {
      chooser.setInitialDirectory(openPath.getParent().toFile());
    }
    File output = chooser.showSaveDialog(stage);
    if (output == null) {
      return;
    }
    OptionalLong request = exportCoordinator.tryBegin();
    if (request.isEmpty()) {
      return;
    }
    long requestGeneration = request.getAsLong();
    refreshExportButtons();
    setBusy(true, "正在导出 " + output.getName() + " …");
    AnalysisResult result = currentResult;
    Task<Void> task =
        new Task<>() {
          @Override
          protected Void call() throws Exception {
            exporter.accept(result, output.toPath());
            return null;
          }
        };
    task.setOnSucceeded(
        ignored -> {
          boolean currentRequest = exportCoordinator.finish(requestGeneration);
          refreshExportButtons();
          if (currentRequest) {
            setBusy(false, "已导出：" + output.toPath().toAbsolutePath());
          }
        });
    task.setOnFailed(
        ignored -> {
          boolean currentRequest = exportCoordinator.finish(requestGeneration);
          refreshExportButtons();
          if (currentRequest) {
            setBusy(false, "导出失败");
            showError("无法导出报告", task.getException());
          }
        });
    task.setOnCancelled(
        ignored -> {
          boolean currentRequest = exportCoordinator.finish(requestGeneration);
          refreshExportButtons();
          if (currentRequest) {
            setBusy(false, "导出已取消");
          }
        });
    startBackground(task, "health-report-export");
  }

  private void setBusy(boolean busy, String status) {
    progress.setVisible(busy);
    progress.setManaged(busy);
    statusLabel.setText(status);
  }

  private void setExportsDisabled(boolean disabled) {
    exportDataAvailable = !disabled;
    refreshExportButtons();
  }

  private void refreshExportButtons() {
    boolean disabled = !exportDataAvailable || exportCoordinator.isInProgress();
    exportJson.setDisable(disabled);
    exportCsv.setDisable(disabled);
    exportHtml.setDisable(disabled);
  }

  private void startBackground(Task<?> task, String name) {
    Thread thread = new Thread(task, name);
    thread.setDaemon(true);
    thread.start();
  }

  private static void cancel(Task<?> task) {
    if (task != null && !task.isDone()) {
      task.cancel(true);
    }
  }

  private void reanalyzeSelectedLog() {
    if (selectedPath != null && selectedLog != null) {
      analyzeLoadedLog(selectedPath, selectedLog);
    } else {
      setExportsDisabled(true);
    }
  }

  private String relativeStart(Anomaly anomaly) {
    if (currentLog == null) {
      return "";
    }
    return formatSeconds((anomaly.startMicros() - currentLog.startMicros()) / 1_000_000.0);
  }

  private static String objectName(Anomaly anomaly) {
    if (!anomaly.motor().isBlank()) {
      return anomaly.subsystem() + "/" + anomaly.motor();
    }
    return anomaly.subsystem().isBlank() ? "Robot" : anomaly.subsystem();
  }

  private static Region separator() {
    Region region = new Region();
    region.setMinWidth(7.0);
    region.setPrefWidth(7.0);
    return region;
  }

  private void showError(String title, Throwable throwable) {
    Alert alert = new Alert(Alert.AlertType.ERROR);
    alert.initOwner(stage);
    alert.setTitle(title);
    alert.setHeaderText(title);
    alert.setContentText(
        throwable == null
            ? "未知错误"
            : throwable.getClass().getSimpleName() + ": " + throwable.getMessage());
    alert.showAndWait();
  }

  private void showInformation(String title, String content) {
    Alert alert = new Alert(Alert.AlertType.INFORMATION);
    alert.initOwner(stage);
    alert.setTitle(title);
    alert.setHeaderText(title);
    alert.setContentText(content);
    alert.showAndWait();
  }

  private static String format(double value, String suffix) {
    return Double.isFinite(value)
        ? String.format(Locale.ROOT, "%,.3f%s", value, suffix)
        : "Unavailable";
  }

  private static String formatSeconds(double value) {
    return format(value, " s");
  }

  @FunctionalInterface
  private interface BiConsumerWithIOException<T, U> {
    void accept(T first, U second) throws IOException;
  }

  static final class ExportCoordinator {
    private long generation;
    private Long activeGeneration;

    OptionalLong tryBegin() {
      if (activeGeneration != null) {
        return OptionalLong.empty();
      }
      activeGeneration = ++generation;
      return OptionalLong.of(activeGeneration);
    }

    void invalidate() {
      ++generation;
    }

    boolean finish(long requestGeneration) {
      if (activeGeneration == null || activeGeneration.longValue() != requestGeneration) {
        return false;
      }
      activeGeneration = null;
      return requestGeneration == generation;
    }

    boolean isInProgress() {
      return activeGeneration != null;
    }
  }

  private record SessionChoice(
      String label, long startMicros, long endMicros, boolean fullLog) {
    HealthLog select(HealthLog source) {
      return fullLog ? source : source.slice(startMicros, endMicros);
    }

    @Override
    public String toString() {
      return label;
    }
  }

  private static final class SummaryCards {
    private final TilePane view = new TilePane();
    private final SummaryCard duration = new SummaryCard("测试总时间");
    private final SummaryCard enabled = new SummaryCard("Enabled 时间");
    private final SummaryCard startingVoltage = new SummaryCard("起始电压");
    private final SummaryCard minimumVoltage = new SummaryCard("最低电压");
    private final SummaryCard rawCurrent = new SummaryCard("总电流原始峰值");
    private final SummaryCard rollingCurrent = new SummaryCard("100 ms 峰值");
    private final SummaryCard percentiles = new SummaryCard("P95 / P99 总电流");
    private final SummaryCard ampHours = new SummaryCard("消耗 Ah");
    private final SummaryCard wattHours = new SummaryCard("消耗 Wh");
    private final SummaryCard brownouts = new SummaryCard("Brownout 次数");
    private final SummaryCard anomalies = new SummaryCard("异常数量");

    SummaryCards() {
      view.setHgap(8.0);
      view.setVgap(8.0);
      view.setPadding(new Insets(5.0, 14.0, 11.0, 14.0));
      view.setPrefColumns(11);
      view.setTileAlignment(Pos.CENTER_LEFT);
      view.getChildren()
          .addAll(
              duration.view,
              enabled.view,
              startingVoltage.view,
              minimumVoltage.view,
              rawCurrent.view,
              rollingCurrent.view,
              percentiles.view,
              ampHours.view,
              wattHours.view,
              brownouts.view,
              anomalies.view);
    }

    TilePane view() {
      return view;
    }

    void update(AnalysisSummary summary, HealthLog log) {
      CurrentStatistics current = summary.pdhTotalCurrent();
      String unavailablePowerValue =
          powerDistributionExplicitlyDisabled(log) ? "PDH未配置" : "Unavailable";
      duration.set(formatSeconds(summary.testDurationSeconds()));
      enabled.set(formatSeconds(summary.enabledSeconds()));
      startingVoltage.set(format(summary.startingVoltage(), " V"));
      minimumVoltage.set(format(summary.minimumVoltage(), " V"));
      rawCurrent.set(formatOr(current.rawPeakAmps(), " A", unavailablePowerValue));
      rollingCurrent.set(
          formatOr(current.rolling100msPeakAmps(), " A", unavailablePowerValue));
      percentiles.set(
          formatOr(current.p95Amps(), " A", unavailablePowerValue)
              + " / "
              + formatOr(current.p99Amps(), " A", unavailablePowerValue));
      ampHours.set(formatOr(summary.consumedAmpHours(), " Ah", unavailablePowerValue));
      wattHours.set(formatOr(summary.consumedWattHours(), " Wh", unavailablePowerValue));
      brownouts.set(
          summary.brownoutCount() == null
              ? "Unavailable"
              : Integer.toString(summary.brownoutCount()));
      anomalies.set(Integer.toString(summary.anomalyCount()));
    }

    private static boolean powerDistributionExplicitlyDisabled(HealthLog log) {
      return log != null
          && log.latestBoolean("/Health/Power/DistributionEnabled", log.endMicros())
              .map(sample -> !sample.value())
              .orElse(false);
    }

    private static String formatOr(double value, String suffix, String unavailableValue) {
      return Double.isFinite(value) ? format(value, suffix) : unavailableValue;
    }
  }

  private static final class SummaryCard {
    private final VBox view = new VBox(3.0);
    private final Label value = new Label("—");

    SummaryCard(String title) {
      Label titleLabel = new Label(title);
      titleLabel.getStyleClass().add("summary-title");
      value.getStyleClass().add("summary-value");
      view.getChildren().addAll(titleLabel, value);
      view.getStyleClass().add("summary-card");
      view.setMinWidth(132.0);
    }

    void set(String newValue) {
      value.setText(newValue);
    }
  }
}
