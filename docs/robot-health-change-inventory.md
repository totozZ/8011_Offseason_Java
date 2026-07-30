# 机器人健康日志改动清单

本文列出当前工作区中与机器人健康日志及桌面分析器有关的全部源码、测试和文档改动。
Gradle 的 `build/` 输出未列入版本化文件。

## 用户原有改动

- `src/main/java/frc/robot/Constants.java`：工作开始前已有
  `intakePitchMotorMaxPositionRot` 从 `8.2` 改为 `7` 的控制参数变更。本次工作保留它，
  不把它归入健康日志实现。

## 机器人端既有文件

- `src/main/java/frc/robot/Robot.java`：初始化 logger，记录模式、会话、事件、循环耗时。
- `src/main/java/frc/robot/RobotContainer.java`：集中注册各子系统健康数据源。
- `src/main/java/frc/robot/Telemetry.java`：移除重复的 Phoenix 日志所有权，交由健康
  logger 管理。
- `src/main/java/frc/robot/frc8011/WayiMotor.java`：提供不触发额外刷新且会检查状态和
  时间戳的缓存速度/位置读数。
- `src/main/java/frc/robot/subsystems/ClientSubsystem.java`：暴露健康状态和活动状态。
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`：注册八台驱动电机及
  Drive 参考量、测量量和状态。
- `src/main/java/frc/robot/subsystems/FeederSubsystem.java`：注册双电机、需求、速度和
  活动状态。
- `src/main/java/frc/robot/subsystems/GroundIntakeSubsystem.java`：注册滚轮/俯仰电机、
  限位、目标和测量状态。
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`：注册五台电机、目标速度、
  测量速度和工作状态。
- `src/main/java/frc/robot/subsystems/VisionSubsystem.java`：记录相机连接和视觉状态。

## 机器人端新增文件

- `src/main/java/frc/robot/logging/HealthConfig.java`：安全默认值、采样周期、Phoenix、
  PDH/PDP 和日志策略配置。
- `src/main/java/frc/robot/logging/MotorHealthConfig.java`：单电机信号选择、角色和
  采样配置。
- `src/main/java/frc/robot/logging/PdhChannelMap.java`：显式且可验证的配电通道映射。
- `src/main/java/frc/robot/logging/RobotHealthLogger.java`：WPILOG/Hoot 生命周期、
  会话、事件、命令、机器人/CAN/电源、子系统和 TalonFX 健康采集核心。
- `src/test/java/frc/robot/logging/HealthConfigTest.java`：安全默认配置和配电映射测试。
- `docs/robot-health-testing.zh-CN.md`：采集、下载、查看、分析、阈值、验收和限制说明。
- `docs/robot-health-change-inventory.md`：本清单。

## 独立桌面分析器工程

### 工程与文档

- `tools/health-analyzer/.gitignore`
- `tools/health-analyzer/README.md`
- `tools/health-analyzer/build.gradle`
- `tools/health-analyzer/settings.gradle`

### 分析与统计

- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/AnalysisResult.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/AnalysisSummary.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/CurrentStatistics.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/HealthAnalyzer.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/HealthSignalIndex.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/MotorAnalysis.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/SeriesMath.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/SignalStatistics.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/SubsystemAnalysis.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/TelemetryCoverage.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/analysis/TrackingErrorSeries.java`

### 启动、加载与数据模型

- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/app/HealthAnalyzerApp.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/app/HealthAnalyzerLauncher.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/io/WpiLogLoader.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/io/wpilog/PureJavaWpiLogWriter.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/HealthEvent.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/HealthLog.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/RobotModeInterval.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/Sample.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/SignalType.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/model/TestSessionInterval.java`

### 规则、报告、样例与界面

- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/report/ReportExporter.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/Anomaly.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/HealthRules.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/RuleConfig.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/RuleEngine.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/RuleType.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/rules/Severity.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/sample/SampleHealthLogGenerator.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/ui/SignalSelectorPane.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/ui/SnapshotPane.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/ui/TimelineChartPane.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/ui/TimelineMarker.java`
- `tools/health-analyzer/src/main/java/frc/robot/health/analyzer/ui/TimelineViewport.java`
- `tools/health-analyzer/src/main/resources/health-rules.json`
- `tools/health-analyzer/src/main/resources/ui/health-analyzer.css`

### 分析器测试

- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/analysis/HealthAnalyzerTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/analysis/SeriesMathTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/analysis/SignalStatisticsTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/analysis/TelemetryCoverageTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/app/HealthAnalyzerAppTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/app/HealthAnalyzerLauncherTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/io/WpiLogLoaderTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/io/wpilog/PureJavaWpiLogWriterTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/model/HealthLogTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/report/ReportExporterTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/rules/HealthRulesTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/rules/RuleEngineTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/sample/SampleHealthLogGeneratorTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/ui/SnapshotPaneTest.java`
- `tools/health-analyzer/src/test/java/frc/robot/health/analyzer/ui/TimelineViewportTest.java`

## 明确未改动

- `src/main/java/frc/robot/generated/TunerConstants.java`：遵守生成文件边界，未手工修改。
- 根工程 `build.gradle`、`settings.gradle`：分析器保持独立，JavaFX/Gson 不进入
  roboRIO 工程。
