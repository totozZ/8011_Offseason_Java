package frc.robot.health.analyzer.ui;

import frc.robot.health.analyzer.model.HealthLog;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Predicate;
import javafx.geometry.Insets;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.OverrunStyle;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;

/** Filterable signal chooser with practical whole-robot and mechanism presets. */
public final class SignalSelectorPane extends VBox {
  private final TextField filter = new TextField();
  private final VBox signalList = new VBox(3.0);
  private final Label selectionCount = new Label("未加载日志");
  private final Set<String> selected = new LinkedHashSet<>();
  private List<String> available = List.of();
  private Consumer<List<String>> selectionListener = ignored -> {};

  public SignalSelectorPane() {
    setSpacing(9.0);
    setPadding(new Insets(10.0));
    getStyleClass().add("side-panel");
    setPrefWidth(286.0);
    setMinWidth(220.0);

    Label title = new Label("信号");
    title.getStyleClass().add("panel-title");

    filter.setPromptText("筛选 /Health 路径");
    filter.textProperty().addListener((ignored, oldValue, newValue) -> rebuild());

    FlowPane presets = new FlowPane(6.0, 6.0);
    presets.getChildren()
        .addAll(
            presetButton(
                "整机",
                path ->
                    path.equals("/Health/Power/Voltage")
                        || path.equals("/Health/Power/TotalCurrent")
                        || path.equals("/Health/Robot/LoopTimeMs")
                        || path.equals("/Health/Robot/CAN/Utilization")),
            presetButton(
                "子系统",
                path ->
                    path.startsWith("/Health/Subsystems/")
                        && (path.endsWith("/SupplyCurrent")
                            || path.endsWith("/Reference")
                            || path.endsWith("/Measured"))),
            presetButton(
                "电机电流",
                path ->
                    path.startsWith("/Health/Motors/")
                        && (path.endsWith("/SupplyCurrent")
                            || path.endsWith("/StatorCurrent"))),
            presetButton(
                "目标/实际",
                path ->
                    path.endsWith("/Reference")
                        || path.endsWith("/Velocity")
                        || path.endsWith("/Measured")),
            presetButton(
                "温度/CAN",
                path ->
                    path.endsWith("/Temperature")
                        || path.startsWith("/Health/Robot/CAN/")));
    Button clear = new Button("清空");
    clear.getStyleClass().add("compact-button");
    clear.setOnAction(
        ignored -> {
          selected.clear();
          rebuild();
          publishSelection();
        });
    presets.getChildren().add(clear);

    ScrollPane scrollPane = new ScrollPane(signalList);
    scrollPane.setFitToWidth(true);
    scrollPane.setHbarPolicy(ScrollPane.ScrollBarPolicy.NEVER);
    scrollPane.setMinWidth(0.0);
    signalList.setMinWidth(0.0);
    VBox.setVgrow(scrollPane, Priority.ALWAYS);

    selectionCount.getStyleClass().add("muted-label");
    getChildren().addAll(title, filter, presets, scrollPane, selectionCount);
  }

  public void setLog(HealthLog log) {
    available =
        log == null
            ? List.of()
            : log.doubleSignals().keySet().stream().sorted().toList();
    selected.clear();
    if (log != null) {
      selectMatching(
          path ->
              path.equals("/Health/Power/Voltage")
                  || path.equals("/Health/Power/TotalCurrent")
                  || path.equals("/Health/Robot/CAN/Utilization")
                  || (path.startsWith("/Health/Subsystems/")
                      && path.endsWith("/SupplyCurrent")),
          12);
    }
    rebuild();
    publishSelection();
  }

  public void setOnSelectionChanged(Consumer<List<String>> listener) {
    selectionListener = listener == null ? ignored -> {} : listener;
  }

  public List<String> selectedSignals() {
    return List.copyOf(selected);
  }

  private Button presetButton(String label, Predicate<String> predicate) {
    Button button = new Button(label);
    button.getStyleClass().add("compact-button");
    button.setOnAction(
        ignored -> {
          selected.clear();
          selectMatching(predicate, Integer.MAX_VALUE);
          rebuild();
          publishSelection();
        });
    return button;
  }

  private void selectMatching(Predicate<String> predicate, int maximum) {
    int added = 0;
    for (String path : available) {
      if (predicate.test(path) && added < maximum) {
        selected.add(path);
        added++;
      }
    }
  }

  private void rebuild() {
    signalList.getChildren().clear();
    String needle = filter.getText() == null ? "" : filter.getText().trim().toLowerCase(Locale.ROOT);
    int visible = 0;
    for (String path : available) {
      if (!needle.isEmpty() && !path.toLowerCase(Locale.ROOT).contains(needle)) {
        continue;
      }
      visible++;
      CheckBox checkBox = new CheckBox(path);
      checkBox.setSelected(selected.contains(path));
      checkBox.setMinWidth(0.0);
      checkBox.setMaxWidth(Double.MAX_VALUE);
      checkBox.setTextOverrun(OverrunStyle.LEADING_ELLIPSIS);
      checkBox.setTooltip(new Tooltip(path));
      checkBox.getStyleClass().add("signal-checkbox");
      checkBox
          .selectedProperty()
          .addListener(
              (ignored, wasSelected, isSelected) -> {
                if (isSelected) {
                  selected.add(path);
                } else {
                  selected.remove(path);
                }
                updateCount();
                publishSelection();
              });
      signalList.getChildren().add(checkBox);
    }
    if (available.isEmpty()) {
      signalList.getChildren().add(new Label("打开 WPILOG 后显示数值信号"));
    } else if (visible == 0) {
      signalList.getChildren().add(new Label("没有匹配信号"));
    }
    updateCount();
  }

  private void updateCount() {
    if (available.isEmpty()) {
      selectionCount.setText("未加载日志");
    } else {
      selectionCount.setText(
          "已选 %d / %d 个数值信号".formatted(selected.size(), available.size()));
    }
  }

  private void publishSelection() {
    selectionListener.accept(new ArrayList<>(selected));
  }
}
