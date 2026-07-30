// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Direct-to-WPILOG robot health recorder.
 *
 * <p>Call {@link #periodic()} once from the robot main loop. All Phoenix status
 * signal objects are cached during registration and refreshed in non-blocking,
 * same-CAN-bus batches. Runtime failures are contained inside this class so a
 * disconnected device or logging failure cannot stop robot control.
 */
public final class RobotHealthLogger implements AutoCloseable {
    private static final String ROOT = "/Health";

    private final HealthConfig config;
    private final Map<String, MotorRegistration> motors = new LinkedHashMap<>();
    private final Map<String, SubsystemRegistration> subsystems = new LinkedHashMap<>();
    private final Map<String, StringLogEntry> stateChangeEntries = new LinkedHashMap<>();
    private final Map<String, List<BaseStatusSignal>> fastSignalsByNetwork =
            new LinkedHashMap<>();
    private final Map<String, List<BaseStatusSignal>> currentSignalsByNetwork =
            new LinkedHashMap<>();
    private final Map<String, List<BaseStatusSignal>> slowSignalsByNetwork =
            new LinkedHashMap<>();
    private final Map<String, CanBusRegistration> canBuses = new LinkedHashMap<>();

    private DataLog log;
    private PowerDistribution powerDistribution;
    private DoubleLogEntry powerVoltageEntry;
    private DoubleLogEntry powerTotalCurrentEntry;
    private DoubleLogEntry powerTotalPowerEntry;
    private DoubleLogEntry powerTotalEnergyEntry;
    private DoubleLogEntry powerTemperatureEntry;
    private BooleanLogEntry powerDistributionEnabledEntry;
    private StringLogEntry powerDistributionTypeEntry;
    private IntegerLogEntry powerDistributionModuleEntry;
    private DoubleLogEntry[] pdhCurrentEntries = new DoubleLogEntry[0];
    private double[] pdhCurrents = new double[0];
    private int pdhHardwareChannelCount;
    private boolean pdhUpdatedThisCycle;

    private BooleanLogEntry robotEnabledEntry;
    private StringLogEntry robotModeEntry;
    private BooleanLogEntry robotBrownedOutEntry;
    private DoubleLogEntry robotLoopTimeEntry;
    private DoubleLogEntry canUtilizationEntry;
    private IntegerLogEntry canBusOffEntry;
    private IntegerLogEntry canReceiveErrorsEntry;
    private IntegerLogEntry canTransmitErrorsEntry;
    private IntegerLogEntry canTxFullEntry;
    private StringLogEntry eventCategoryEntry;
    private StringLogEntry eventMessageEntry;
    private BooleanLogEntry sessionActiveEntry;
    private StringLogEntry sessionNameEntry;
    private IntegerLogEntry loggerErrorCountEntry;
    private StringLogEntry loggerLastErrorEntry;
    private DoubleLogEntry loggerPeriodicTimeEntry;

    private boolean initialized;
    private boolean closed;
    private boolean sessionActive;
    private boolean signalLoggerRunning;
    private boolean lastBrownedOut;
    private String lastMode = "";
    private String sessionName = "";
    private long loggerErrorCount;
    private double lastPeriodicTimestamp = Double.NaN;
    private double nextRobotSample;
    private double nextCanSample;
    private double nextSubsystemSample;
    private double nextPdhSample;
    private double nextMotorFastSample;
    private double nextMotorCurrentSample;
    private double nextMotorSlowSample;
    private double nextFlush;

    public RobotHealthLogger() {
        this(HealthConfig.defaults());
    }

    public RobotHealthLogger(HealthConfig config) {
        this.config = config == null ? HealthConfig.defaults() : config;
        initialize();
    }

    /** Starts WPILOG/Driver Station logging and installs command lifecycle hooks. */
    public synchronized void initialize() {
        if (initialized || closed) {
            return;
        }
        try {
            // High-rate health samples use direct LogEntry objects. NetworkTables can
            // contain 250 Hz drivetrain telemetry, so it is opt-in.
            DataLogManager.logNetworkTables(config.logNetworkTables());
            DataLogManager.start();
            log = DataLogManager.getLog();
            DriverStation.startDataLog(log, config.logJoysticks());
            createCoreEntries();
            initialized = true;
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("initialize-wpilog", ex);
            return;
        }

        if (config.startSignalLogger()) {
            safeRun("initialize-signal-logger", () -> {
                // Capture startup configuration before RobotBoot begins. After the
                // first stop, explicit test-session start/stop owns the Hoot lifecycle.
                // Disabling Phoenix auto-start prevents a DS connection from silently
                // restarting capture outside that lifecycle.
                checkStatus(
                        "signal-logger-auto-start",
                        SignalLogger.enableAutoLogging(false));
                startSignalLogger("signal-logger-start");
            });
        }
        safeRun("initialize-power-distribution", this::initializePowerDistribution);
        installCommandLifecycleHooks();
        update(powerDistributionEnabledEntry, powerDistribution != null);
        update(sessionActiveEntry, false);
        update(sessionNameEntry, "");
        markEvent("Logger", "Robot health logger initialized");
    }

    /**
     * Registers a TalonFX with default signal selection and independent role.
     *
     * @return true when registration succeeded
     */
    public boolean registerTalonFX(String subsystem, String motorName, TalonFX motor) {
        return registerTalonFX(subsystem, motorName, motor, MotorHealthConfig.defaults());
    }

    /**
     * Registers and caches all selected TalonFX status signals. No reflection is
     * used, and duplicate health paths are rejected.
     *
     * @return true when registration succeeded
     */
    public synchronized boolean registerTalonFX(
            String subsystem,
            String motorName,
            TalonFX motor,
            MotorHealthConfig motorConfig) {
        if (!initialized || closed || motor == null) {
            return false;
        }
        String cleanSubsystem = pathSegment(subsystem);
        String cleanMotor = pathSegment(motorName);
        String key = cleanSubsystem + "/" + cleanMotor;
        if (motors.containsKey(key)) {
            markEvent("Logger", "Duplicate motor registration ignored: " + key);
            return false;
        }

        try {
            MotorRegistration registration = new MotorRegistration(
                    cleanSubsystem,
                    cleanMotor,
                    motor,
                    motorConfig == null ? MotorHealthConfig.defaults() : motorConfig);
            motors.put(key, registration);
            addSignals(fastSignalsByNetwork, registration.network, registration.fastSignals);
            addSignals(currentSignalsByNetwork, registration.network, registration.currentSignals);
            addSignals(slowSignalsByNetwork, registration.network, registration.slowSignals);
            canBuses.computeIfAbsent(
                    registration.network,
                    ignored -> new CanBusRegistration(motor.getNetwork()));
            configureSignalFrequencies(registration);
            registration.writeMetadata();
            markEvent(
                    "MotorRegistered",
                    cleanSubsystem + "/" + cleanMotor
                            + " CAN=" + registration.network
                            + " ID=" + motor.getDeviceID());
            return true;
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("register-motor-" + key, ex);
            return false;
        }
    }

    /**
     * Registers control context and optional PDH channels for a subsystem.
     *
     * <p>If no channels are supplied, channels explicitly mapped to the same
     * subsystem name in {@link PdhChannelMap} are used. Unknown channels are
     * never inferred. Supply current prefers mapped PDH channels and otherwise
     * falls back to the sum of registered TalonFX supply currents.
     */
    public synchronized boolean registerSubsystem(
            String name,
            Subsystem subsystem,
            BooleanSupplier active,
            Supplier<String> state,
            DoubleSupplier reference,
            DoubleSupplier measured,
            int... pdhChannels) {
        if (!initialized || closed) {
            return false;
        }
        String cleanName = pathSegment(name);
        if (subsystems.containsKey(cleanName)) {
            markEvent("Logger", "Duplicate subsystem registration ignored: " + cleanName);
            return false;
        }
        try {
            int[] channels = pdhChannels == null || pdhChannels.length == 0
                    ? config.pdhChannelMap().channelsForSubsystem(name)
                    : Arrays.copyOf(pdhChannels, pdhChannels.length);
            SubsystemRegistration registration = new SubsystemRegistration(
                    cleanName,
                    subsystem,
                    active == null ? () -> false : active,
                    state == null ? () -> "Unavailable" : state,
                    reference,
                    measured,
                    channels);
            subsystems.put(cleanName, registration);
            registration.currentSourceEntry.update(registration.supplyCurrentSource());
            markEvent("SubsystemRegistered", cleanName);
            return true;
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("register-subsystem-" + cleanName, ex);
            return false;
        }
    }

    /** Convenience overload for a subsystem that only needs state/active logging. */
    public boolean registerSubsystem(
            String name,
            Subsystem subsystem,
            BooleanSupplier active,
            Supplier<String> state,
            int... pdhChannels) {
        return registerSubsystem(
                name,
                subsystem,
                active,
                state,
                null,
                null,
                pdhChannels);
    }

    /** Marks the beginning of a named test interval in WPILOG and Hoot. */
    public synchronized void startTestSession(String name) {
        if (!initialized || closed) {
            return;
        }
        String cleanName = name == null || name.isBlank() ? "Unnamed" : name.trim();
        if (sessionActive) {
            markEvent("TestSession", "Restarting active session " + sessionName);
            stopTestSession();
        }
        sessionName = cleanName;
        sessionActive = true;
        for (SubsystemRegistration subsystem : subsystems.values()) {
            subsystem.resetSessionEnergy();
        }
        update(sessionNameEntry, cleanName);
        update(sessionActiveEntry, true);
        if (config.startSignalLogger()) {
            safeRun(
                    "start-session-signal-logger",
                    () -> startSignalLogger("signal-logger-start-session"));
        }
        markEvent("TestSessionStart", cleanName);
    }

    /** Marks the end of the current test, flushes WPILOG, and stops Hoot capture. */
    public synchronized void stopTestSession() {
        if (!initialized || closed || !sessionActive) {
            return;
        }
        markEvent("TestSessionStop", sessionName);
        sessionActive = false;
        update(sessionActiveEntry, false);
        if (config.startSignalLogger()) {
            safeRun(
                    "stop-session-signal-logger",
                    () -> stopSignalLogger("signal-logger-stop-session"));
        }
        flush();
    }

    /** Appends an event marker. Category and message share the same log timestamp. */
    public synchronized void markEvent(String category, String message) {
        if (!initialized || closed) {
            return;
        }
        String cleanCategory =
                category == null || category.isBlank() ? "General" : category.trim();
        String cleanMessage = message == null ? "" : message;
        try {
            long timestamp = (long) (Timer.getFPGATimestamp() * 1_000_000.0);
            eventCategoryEntry.append(cleanCategory, timestamp);
            eventMessageEntry.append(cleanMessage, timestamp);
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("event", ex);
        }
    }

    /** Records a named string only when its value changes. */
    public synchronized void logStateChange(String key, String value) {
        if (!initialized || closed) {
            return;
        }
        try {
            String cleanKey = pathSegment(key);
            StringLogEntry entry = stateChangeEntries.computeIfAbsent(
                    cleanKey,
                    ignored -> new StringLogEntry(log, ROOT + "/StateChanges/" + cleanKey));
            entry.update(value == null ? "Unavailable" : value);
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("state-change", ex);
        }
    }

    /** Records a subsystem-scoped string only when its value changes. */
    public void logStateChange(String subsystem, String key, String value) {
        logStateChange(pathSegment(subsystem) + "/" + pathSegment(key), value);
    }

    /**
     * Samples health data. The loop time is estimated from the interval between
     * calls; use {@link #periodic(double)} when the caller has a measured value.
     */
    public void periodic() {
        double now;
        try {
            now = Timer.getFPGATimestamp();
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("periodic-clock", ex);
            return;
        }
        double loopTimeMs = Double.isFinite(lastPeriodicTimestamp)
                ? (now - lastPeriodicTimestamp) * 1000.0
                : Double.NaN;
        periodicInternal(now, loopTimeMs);
    }

    /** Samples health data using a caller-measured robot-loop duration. */
    public void periodic(double robotLoopTimeMs) {
        try {
            periodicInternal(Timer.getFPGATimestamp(), robotLoopTimeMs);
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("periodic", ex);
        }
    }

    private synchronized void periodicInternal(double now, double robotLoopTimeMs) {
        if (!initialized || closed) {
            return;
        }
        long loggerStartNanos = System.nanoTime();
        try {
            lastPeriodicTimestamp = now;
            pdhUpdatedThisCycle = false;
            for (MotorRegistration motor : motors.values()) {
                motor.supplyCurrentUpdatedThisCycle = false;
                motor.statorCurrentUpdatedThisCycle = false;
            }

            logRobotStateChanges();
            logSubsystemStateChanges();

            if (now >= nextRobotSample) {
                nextRobotSample =
                        advanceNextSample(nextRobotSample, now, config.robotSampleHz());
                appendFinite(robotLoopTimeEntry, robotLoopTimeMs);
                appendFinite(powerVoltageEntry, RobotController.getBatteryVoltage());
            }
            if (sessionActive && now >= nextPdhSample) {
                nextPdhSample =
                        advanceNextSample(nextPdhSample, now, config.pdhSampleHz());
                samplePowerDistribution();
            }
            if (now >= nextMotorSlowSample) {
                nextMotorSlowSample =
                        advanceNextSample(
                                nextMotorSlowSample, now, config.motorSlowSampleHz());
                refreshBatches(slowSignalsByNetwork);
                for (MotorRegistration motor : motors.values()) {
                    safeRun(
                            "motor-slow-" + motor.subsystem + "/" + motor.name,
                            motor::logSlow);
                }
            }
            if (sessionActive && now >= nextMotorFastSample) {
                nextMotorFastSample =
                        advanceNextSample(
                                nextMotorFastSample, now, config.motorFastSampleHz());
                refreshBatches(fastSignalsByNetwork);
                for (MotorRegistration motor : motors.values()) {
                    safeRun(
                            "motor-fast-" + motor.subsystem + "/" + motor.name,
                            motor::logFast);
                }
            }
            if (sessionActive && now >= nextMotorCurrentSample) {
                nextMotorCurrentSample =
                        advanceNextSample(
                                nextMotorCurrentSample, now, config.motorCurrentSampleHz());
                refreshBatches(currentSignalsByNetwork);
                for (MotorRegistration motor : motors.values()) {
                    safeRun(
                            "motor-current-" + motor.subsystem + "/" + motor.name,
                            motor::logCurrents);
                }
            }
            if (sessionActive && now >= nextSubsystemSample) {
                nextSubsystemSample =
                        advanceNextSample(
                                nextSubsystemSample, now, config.subsystemSampleHz());
                for (SubsystemRegistration subsystem : subsystems.values()) {
                    safeRun(
                            "subsystem-numeric-" + subsystem.name,
                            () -> subsystem.logNumeric(now));
                }
            }
            if (now >= nextCanSample) {
                nextCanSample =
                        advanceNextSample(nextCanSample, now, config.canSampleHz());
                sampleCanStatus();
            }
            if (now >= nextFlush) {
                nextFlush = now + config.flushPeriodSeconds();
                flush();
            }
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("periodic-body", ex);
        } finally {
            try {
                appendFinite(
                        loggerPeriodicTimeEntry,
                        (System.nanoTime() - loggerStartNanos) / 1_000_000.0);
            } catch (RuntimeException | LinkageError ex) {
                noteFailure("periodic-duration", ex);
            }
        }
    }

    /** Flushes buffered WPILOG data without stopping robot logging. */
    public synchronized void flush() {
        if (!initialized || log == null) {
            return;
        }
        try {
            log.flush();
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("flush", ex);
        }
    }

    @Override
    public synchronized void close() {
        if (closed) {
            return;
        }
        if (sessionActive) {
            stopTestSession();
        }
        markEvent("Logger", "Robot health logger closing");
        flush();
        if (config.startSignalLogger()) {
            safeRun(
                    "close-signal-logger",
                    () -> stopSignalLogger("signal-logger-stop-close"));
        }
        if (powerDistribution != null) {
            safeRun("close-power-distribution", powerDistribution::close);
            powerDistribution = null;
        }
        if (config.stopDataLogOnClose()) {
            safeRun("close-wpilog", DataLogManager::stop);
        }
        closed = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public boolean isSessionActive() {
        return sessionActive;
    }

    public int registeredMotorCount() {
        return motors.size();
    }

    public int registeredSubsystemCount() {
        return subsystems.size();
    }

    private void createCoreEntries() {
        powerVoltageEntry = new DoubleLogEntry(log, ROOT + "/Power/Voltage");
        powerTotalCurrentEntry = new DoubleLogEntry(log, ROOT + "/Power/TotalCurrent");
        powerTotalPowerEntry = new DoubleLogEntry(log, ROOT + "/Power/TotalPower");
        powerTotalEnergyEntry = new DoubleLogEntry(log, ROOT + "/Power/TotalEnergy");
        powerTemperatureEntry = new DoubleLogEntry(log, ROOT + "/Power/Temperature");
        powerDistributionEnabledEntry =
                new BooleanLogEntry(log, ROOT + "/Power/DistributionEnabled");
        powerDistributionTypeEntry =
                new StringLogEntry(log, ROOT + "/Power/DistributionType");
        powerDistributionModuleEntry =
                new IntegerLogEntry(log, ROOT + "/Power/DistributionModule");

        robotEnabledEntry = new BooleanLogEntry(log, ROOT + "/Robot/Enabled");
        robotModeEntry = new StringLogEntry(log, ROOT + "/Robot/Mode");
        robotBrownedOutEntry = new BooleanLogEntry(log, ROOT + "/Robot/BrownedOut");
        robotLoopTimeEntry = new DoubleLogEntry(log, ROOT + "/Robot/LoopTimeMs");
        canUtilizationEntry = new DoubleLogEntry(log, ROOT + "/Robot/CAN/Utilization");
        canBusOffEntry = new IntegerLogEntry(log, ROOT + "/Robot/CAN/BusOffCount");
        canReceiveErrorsEntry =
                new IntegerLogEntry(log, ROOT + "/Robot/CAN/ReceiveErrors");
        canTransmitErrorsEntry =
                new IntegerLogEntry(log, ROOT + "/Robot/CAN/TransmitErrors");
        canTxFullEntry = new IntegerLogEntry(log, ROOT + "/Robot/CAN/TxFullCount");
        eventCategoryEntry = new StringLogEntry(log, ROOT + "/Events/Category");
        eventMessageEntry = new StringLogEntry(log, ROOT + "/Events/Message");
        sessionActiveEntry = new BooleanLogEntry(log, ROOT + "/TestSession/Active");
        sessionNameEntry = new StringLogEntry(log, ROOT + "/TestSession/Name");
        loggerErrorCountEntry = new IntegerLogEntry(log, ROOT + "/Logger/ErrorCount");
        loggerLastErrorEntry = new StringLogEntry(log, ROOT + "/Logger/LastError");
        loggerPeriodicTimeEntry = new DoubleLogEntry(log, ROOT + "/Logger/PeriodicTimeMs");
    }

    private void initializePowerDistribution() {
        if (!config.pdhEnabled()) {
            createPdhChannelEntries(config.pdhChannelMap().channelCount());
            update(powerDistributionEnabledEntry, false);
            update(powerDistributionTypeEntry, "Unavailable");
            update(powerDistributionModuleEntry, -1);
            return;
        }
        try {
            powerDistribution =
                    new PowerDistribution(config.pdhModule(), config.pdhType());
            pdhHardwareChannelCount = powerDistribution.getNumChannels();
            createPdhChannelEntries(Math.max(
                    config.pdhChannelMap().channelCount(),
                    pdhHardwareChannelCount));
            update(powerDistributionEnabledEntry, true);
            update(powerDistributionTypeEntry, powerDistribution.getType().toString());
            update(powerDistributionModuleEntry, powerDistribution.getModule());
        } catch (RuntimeException | LinkageError ex) {
            powerDistribution = null;
            pdhHardwareChannelCount = 0;
            createPdhChannelEntries(config.pdhChannelMap().channelCount());
            update(powerDistributionEnabledEntry, false);
            update(powerDistributionTypeEntry, "Unavailable");
            update(powerDistributionModuleEntry, config.pdhModule());
            noteFailure("initialize-power-distribution", ex);
        }
    }

    private void createPdhChannelEntries(int channelCount) {
        pdhHardwareChannelCount = powerDistribution == null ? 0 : pdhHardwareChannelCount;
        pdhCurrents = new double[channelCount];
        Arrays.fill(pdhCurrents, Double.NaN);
        pdhCurrentEntries = new DoubleLogEntry[channelCount];
        for (int channel = 0; channel < channelCount; channel++) {
            String base = ROOT + "/Power/Channels/"
                    + String.format(Locale.ROOT, "%02d", channel);
            pdhCurrentEntries[channel] = new DoubleLogEntry(log, base + "/Current");
            StringLogEntry deviceEntry = new StringLogEntry(log, base + "/Device");
            StringLogEntry subsystemEntry = new StringLogEntry(log, base + "/Subsystem");
            if (channel < config.pdhChannelMap().channelCount()) {
                deviceEntry.append(config.pdhChannelMap().deviceLabel(channel));
                subsystemEntry.append(config.pdhChannelMap().subsystemLabel(channel));
            } else {
                deviceEntry.append("Unknown/Channel"
                        + String.format(Locale.ROOT, "%02d", channel));
                subsystemEntry.append("Unknown");
            }
        }
    }

    private void installCommandLifecycleHooks() {
        safeRun("install-command-hooks", () -> {
            CommandScheduler scheduler = CommandScheduler.getInstance();
            scheduler.onCommandInitialize(command -> logCommandEvent("Initialize", command, null));
            scheduler.onCommandFinish(command -> logCommandEvent("Finish", command, null));
            scheduler.onCommandInterrupt(
                    (command, interruptor) -> logCommandEvent(
                            "Interrupt",
                            command,
                            interruptor.orElse(null)));
        });
    }

    private void logCommandEvent(String action, Command command, Command interruptor) {
        try {
            StringBuilder message = new StringBuilder(command == null
                    ? "Unknown"
                    : command.getName());
            if (command != null && !command.getRequirements().isEmpty()) {
                message.append(" [");
                boolean first = true;
                for (Subsystem requirement : command.getRequirements()) {
                    if (!first) {
                        message.append(',');
                    }
                    first = false;
                    message.append(requirement.getName());
                }
                message.append(']');
            }
            if (interruptor != null) {
                message.append(" interruptedBy=").append(interruptor.getName());
            }
            markEvent("Command/" + action, message.toString());
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("command-event", ex);
        }
    }

    private void logRobotStateChanges() {
        boolean enabled = DriverStation.isEnabled();
        String mode = robotMode();
        boolean brownedOut = RobotController.isBrownedOut();
        update(robotEnabledEntry, enabled);
        update(robotModeEntry, mode);
        update(robotBrownedOutEntry, brownedOut);
        if (!Objects.equals(lastMode, mode)) {
            markEvent("RobotMode", mode);
            lastMode = mode;
        }
        if (brownedOut && !lastBrownedOut) {
            markEvent("Brownout", "RobotController reported brownout");
        }
        lastBrownedOut = brownedOut;
    }

    private void logSubsystemStateChanges() {
        for (SubsystemRegistration registration : subsystems.values()) {
            registration.logState();
        }
    }

    private void samplePowerDistribution() {
        if (powerDistribution == null) {
            return;
        }
        try {
            double[] currents = powerDistribution.getAllCurrents();
            double totalCurrent = powerDistribution.getTotalCurrent();
            double totalPower = powerDistribution.getTotalPower();
            double totalEnergy = powerDistribution.getTotalEnergy();
            double temperature = powerDistribution.getTemperature();

            // Publish one coherent sample. Clearing first also prevents a shorter
            // or partially unavailable read from retaining older channel values.
            Arrays.fill(pdhCurrents, Double.NaN);
            int count = Math.min(currents.length, pdhCurrentEntries.length);
            for (int channel = 0; channel < count; channel++) {
                pdhCurrents[channel] = currents[channel];
                appendFinite(pdhCurrentEntries[channel], currents[channel]);
            }
            appendFinite(powerTotalCurrentEntry, totalCurrent);
            appendFinite(powerTotalPowerEntry, totalPower);
            appendFinite(powerTotalEnergyEntry, totalEnergy);
            if (supportsPowerDistributionTemperature(powerDistribution.getType())) {
                appendFinite(powerTemperatureEntry, temperature);
            }
            pdhUpdatedThisCycle = true;
        } catch (RuntimeException | LinkageError ex) {
            // Never integrate a previous PDH sample through an outage.
            Arrays.fill(pdhCurrents, Double.NaN);
            pdhUpdatedThisCycle = false;
            noteFailure("sample-power-distribution", ex);
        }
    }

    static boolean supportsPowerDistributionTemperature(
            PowerDistribution.ModuleType moduleType) {
        // WPILib reports 0 for REV PDH temperature because that telemetry is not
        // supported. Do not record the sentinel as a real 0 °C measurement.
        return moduleType == PowerDistribution.ModuleType.kCTRE;
    }

    private void sampleCanStatus() {
        try {
            CANStatus status = RobotController.getCANStatus();
            appendFinite(canUtilizationEntry, status.percentBusUtilization);
            update(canBusOffEntry, status.busOffCount);
            update(canReceiveErrorsEntry, status.receiveErrorCount);
            update(canTransmitErrorsEntry, status.transmitErrorCount);
            update(canTxFullEntry, status.txFullCount);
        } catch (RuntimeException | LinkageError ex) {
            noteFailure("sample-can", ex);
        }
        for (CanBusRegistration bus : canBuses.values()) {
            safeRun("sample-can-" + bus.name, bus::sample);
        }
    }

    private void refreshBatches(Map<String, List<BaseStatusSignal>> signalsByNetwork) {
        for (List<BaseStatusSignal> signals : signalsByNetwork.values()) {
            if (signals.isEmpty()) {
                continue;
            }
            try {
                // reportError=false keeps an intentionally disconnected device from
                // flooding the Driver Station. Individual signal status is checked below.
                BaseStatusSignal.refreshAll(false, signals);
            } catch (RuntimeException | LinkageError ex) {
                noteFailure("refresh-phoenix-signals", ex);
            }
        }
    }

    private void configureSignalFrequencies(MotorRegistration registration) {
        if (!config.configurePhoenixSignalFrequencies()) {
            return;
        }
        safeRun("configure-phoenix-frequencies", () -> {
            if (!registration.fastSignals.isEmpty()) {
                checkStatus(
                        "set-fast-signal-frequency-" + registration.network,
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                config.motorFastSampleHz(),
                                registration.fastSignals));
            }
            if (!registration.currentSignals.isEmpty()) {
                checkStatus(
                        "set-current-signal-frequency-" + registration.network,
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                config.motorCurrentSampleHz(),
                                registration.currentSignals));
            }
            if (!registration.slowSignals.isEmpty()) {
                checkStatus(
                        "set-slow-signal-frequency-" + registration.network,
                        BaseStatusSignal.setUpdateFrequencyForAll(
                                config.motorSlowSampleHz(),
                                registration.slowSignals));
            }
        });
    }

    private static void addSignals(
            Map<String, List<BaseStatusSignal>> target,
            String network,
            List<BaseStatusSignal> signals) {
        if (!signals.isEmpty()) {
            target.computeIfAbsent(network, ignored -> new ArrayList<>()).addAll(signals);
        }
    }

    private static double advanceNextSample(
            double previousDeadline, double now, double frequencyHz) {
        double period = 1.0 / frequencyHz;
        if (!Double.isFinite(previousDeadline)
                || previousDeadline <= 0.0
                || now - previousDeadline > Math.max(1.0, period * 4.0)) {
            return now + period;
        }
        double next = previousDeadline + period;
        if (next <= now) {
            next += (Math.floor((now - next) / period) + 1.0) * period;
        }
        return next;
    }

    private static String robotMode() {
        if (!DriverStation.isEnabled()) {
            return "Disabled";
        }
        if (DriverStation.isAutonomous()) {
            return "Autonomous";
        }
        if (DriverStation.isTeleop()) {
            return "Teleop";
        }
        if (DriverStation.isTest()) {
            return "Test";
        }
        return "Enabled";
    }

    private static String pathSegment(String value) {
        if (value == null || value.isBlank()) {
            return "Unknown";
        }
        String sanitized = value.trim().replaceAll("[^A-Za-z0-9_.-]+", "_");
        return sanitized.isBlank() ? "Unknown" : sanitized;
    }

    private void safeRun(String context, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException | LinkageError ex) {
            noteFailure(context, ex);
        }
    }

    private boolean checkStatus(String context, StatusCode status) {
        if (status == null || !status.isOK()) {
            noteFailure(context, status == null
                    ? new IllegalStateException("Phoenix returned null status")
                    : new IllegalStateException(status.toString()));
            return false;
        }
        return true;
    }

    private void startSignalLogger(String context) {
        if (!signalLoggerRunning && checkStatus(context, SignalLogger.start())) {
            signalLoggerRunning = true;
        }
    }

    private void stopSignalLogger(String context) {
        if (signalLoggerRunning && checkStatus(context, SignalLogger.stop())) {
            signalLoggerRunning = false;
        }
    }

    private synchronized void noteFailure(String context, Throwable error) {
        loggerErrorCount++;
        String detail = context + ": "
                + (error == null
                        ? "Unknown"
                        : error.getClass().getSimpleName() + " - "
                                + String.valueOf(error.getMessage()));
        try {
            update(loggerErrorCountEntry, loggerErrorCount);
            update(loggerLastErrorEntry, detail);
            DataLogManager.log("[RobotHealthLogger] " + detail);
        } catch (RuntimeException | LinkageError ignored) {
            // Logging failures must never escape into the robot control loop.
        }
    }

    private static void appendFinite(DoubleLogEntry entry, double value) {
        if (entry == null || !Double.isFinite(value)) {
            return;
        }
        try {
            entry.append(value);
        } catch (RuntimeException | LinkageError ignored) {
            // Public logging calls are intentionally fail-safe.
        }
    }

    private static void append(StringLogEntry entry, String value) {
        if (entry == null) {
            return;
        }
        try {
            entry.append(value == null ? "" : value);
        } catch (RuntimeException | LinkageError ignored) {
            // Public logging calls are intentionally fail-safe.
        }
    }

    private static void update(BooleanLogEntry entry, boolean value) {
        if (entry == null) {
            return;
        }
        try {
            entry.update(value);
        } catch (RuntimeException | LinkageError ignored) {
            // Public logging calls are intentionally fail-safe.
        }
    }

    private static void update(IntegerLogEntry entry, long value) {
        if (entry == null) {
            return;
        }
        try {
            entry.update(value);
        } catch (RuntimeException | LinkageError ignored) {
            // Public logging calls are intentionally fail-safe.
        }
    }

    private static void update(StringLogEntry entry, String value) {
        if (entry == null) {
            return;
        }
        try {
            entry.update(value == null ? "Unavailable" : value);
        } catch (RuntimeException | LinkageError ignored) {
            // Public logging calls are intentionally fail-safe.
        }
    }

    private static boolean signalGood(BaseStatusSignal signal) {
        return signal != null && signal.getStatus().isOK();
    }

    private final class CanBusRegistration {
        final CANBus bus;
        final String name;
        final DoubleLogEntry utilizationEntry;
        final IntegerLogEntry busOffEntry;
        final IntegerLogEntry txFullEntry;
        final IntegerLogEntry receiveErrorEntry;
        final IntegerLogEntry transmitErrorEntry;
        final BooleanLogEntry connectedEntry;
        final StringLogEntry statusEntry;

        CanBusRegistration(CANBus bus) {
            this.bus = Objects.requireNonNull(bus);
            name = pathSegment(bus.getName());
            String base = ROOT + "/Robot/CAN/Buses/" + name;
            utilizationEntry = new DoubleLogEntry(log, base + "/Utilization");
            busOffEntry = new IntegerLogEntry(log, base + "/BusOffCount");
            txFullEntry = new IntegerLogEntry(log, base + "/TxFullCount");
            receiveErrorEntry =
                    new IntegerLogEntry(log, base + "/ReceiveErrorCount");
            transmitErrorEntry =
                    new IntegerLogEntry(log, base + "/TransmitErrorCount");
            connectedEntry = new BooleanLogEntry(log, base + "/Connected");
            statusEntry = new StringLogEntry(log, base + "/Status");
        }

        void sample() {
            CANBus.CANBusStatus status = bus.getStatus();
            boolean ok = status != null && status.Status != null && status.Status.isOK();
            update(connectedEntry, ok);
            update(statusEntry, status == null || status.Status == null
                    ? "Unavailable"
                    : status.Status.toString());
            if (!ok) {
                return;
            }
            appendFinite(utilizationEntry, status.BusUtilization);
            update(busOffEntry, status.BusOffCount);
            update(txFullEntry, status.TxFullCount);
            update(receiveErrorEntry, status.REC);
            update(transmitErrorEntry, status.TEC);
        }
    }

    private final class MotorRegistration {
        final String subsystem;
        final String name;
        final TalonFX motor;
        final MotorHealthConfig motorConfig;
        final String network;
        final List<BaseStatusSignal> fastSignals = new ArrayList<>();
        final List<BaseStatusSignal> currentSignals = new ArrayList<>();
        final List<BaseStatusSignal> slowSignals = new ArrayList<>();

        final StatusSignal<Voltage> motorVoltage;
        final StatusSignal<AngularVelocity> velocity;
        final StatusSignal<Angle> position;
        final StatusSignal<Double> reference;
        final StatusSignal<Double> closedLoopError;
        final StatusSignal<Current> supplyCurrent;
        final StatusSignal<Current> statorCurrent;
        final StatusSignal<Current> torqueCurrent;
        final StatusSignal<Temperature> temperature;
        final StatusSignal<Integer> faultField;
        final StatusSignal<Integer> stickyFaultField;
        final StatusSignal<Boolean> supplyCurrentLimited;
        final StatusSignal<Boolean> statorCurrentLimited;
        final StatusSignal<ForwardLimitValue> forwardLimit;
        final StatusSignal<ReverseLimitValue> reverseLimit;
        final StatusSignal<ControlModeValue> controlMode;
        final StatusSignal<Integer> version;

        final DoubleLogEntry supplyCurrentEntry;
        final DoubleLogEntry statorCurrentEntry;
        final DoubleLogEntry torqueCurrentEntry;
        final DoubleLogEntry motorVoltageEntry;
        final DoubleLogEntry velocityEntry;
        final DoubleLogEntry positionEntry;
        final DoubleLogEntry temperatureEntry;
        final DoubleLogEntry referenceEntry;
        final DoubleLogEntry closedLoopErrorEntry;
        final BooleanLogEntry connectedEntry;
        final IntegerLogEntry faultsEntry;
        final IntegerLogEntry stickyFaultsEntry;
        final BooleanLogEntry supplyCurrentLimitedEntry;
        final BooleanLogEntry statorCurrentLimitedEntry;
        final BooleanLogEntry forwardLimitEntry;
        final BooleanLogEntry reverseLimitEntry;
        final StringLogEntry controlModeEntry;

        double lastSupplyCurrent = Double.NaN;
        double lastStatorCurrent = Double.NaN;
        boolean supplyCurrentUpdatedThisCycle;
        boolean statorCurrentUpdatedThisCycle;
        boolean connected;

        MotorRegistration(
                String subsystem,
                String name,
                TalonFX motor,
                MotorHealthConfig motorConfig) {
            this.subsystem = subsystem;
            this.name = name;
            this.motor = motor;
            this.motorConfig = motorConfig;
            network = motor.getNetwork().getName();
            String base = ROOT + "/Motors/" + subsystem + "/" + name;

            if (motorConfig.logMotion()) {
                motorVoltage = motor.getMotorVoltage(false);
                velocity = motor.getVelocity(false);
                position = motor.getPosition(false);
                reference = motor.getClosedLoopReference(false);
                closedLoopError = motor.getClosedLoopError(false);
                fastSignals.addAll(List.of(
                        motorVoltage,
                        velocity,
                        position,
                        reference,
                        closedLoopError));
                motorVoltageEntry = new DoubleLogEntry(log, base + "/MotorVoltage");
                velocityEntry = new DoubleLogEntry(log, base + "/Velocity");
                positionEntry = new DoubleLogEntry(log, base + "/Position");
                referenceEntry = new DoubleLogEntry(log, base + "/Reference");
                closedLoopErrorEntry = new DoubleLogEntry(log, base + "/ClosedLoopError");
            } else {
                motorVoltage = null;
                velocity = null;
                position = null;
                reference = null;
                closedLoopError = null;
                motorVoltageEntry = null;
                velocityEntry = null;
                positionEntry = null;
                referenceEntry = null;
                closedLoopErrorEntry = null;
            }

            if (motorConfig.logElectrical()) {
                supplyCurrent = motor.getSupplyCurrent(false);
                statorCurrent = motor.getStatorCurrent(false);
                torqueCurrent = motor.getTorqueCurrent(false);
                currentSignals.addAll(List.of(supplyCurrent, statorCurrent, torqueCurrent));
                supplyCurrentEntry = new DoubleLogEntry(log, base + "/SupplyCurrent");
                statorCurrentEntry = new DoubleLogEntry(log, base + "/StatorCurrent");
                torqueCurrentEntry = new DoubleLogEntry(log, base + "/TorqueCurrent");
            } else {
                supplyCurrent = null;
                statorCurrent = null;
                torqueCurrent = null;
                supplyCurrentEntry = null;
                statorCurrentEntry = null;
                torqueCurrentEntry = null;
            }

            if (motorConfig.logTemperature()) {
                temperature = motor.getDeviceTemp(false);
                slowSignals.add(temperature);
                temperatureEntry = new DoubleLogEntry(log, base + "/Temperature");
            } else {
                temperature = null;
                temperatureEntry = null;
            }

            if (motorConfig.logFaults()) {
                faultField = motor.getFaultField(false);
                stickyFaultField = motor.getStickyFaultField(false);
                supplyCurrentLimited = motor.getFault_SupplyCurrLimit(false);
                statorCurrentLimited = motor.getFault_StatorCurrLimit(false);
                slowSignals.addAll(List.of(
                        faultField,
                        stickyFaultField,
                        supplyCurrentLimited,
                        statorCurrentLimited));
                faultsEntry = new IntegerLogEntry(log, base + "/Faults");
                stickyFaultsEntry = new IntegerLogEntry(log, base + "/StickyFaults");
                supplyCurrentLimitedEntry =
                        new BooleanLogEntry(log, base + "/SupplyCurrentLimited");
                statorCurrentLimitedEntry =
                        new BooleanLogEntry(log, base + "/StatorCurrentLimited");
            } else {
                faultField = null;
                stickyFaultField = null;
                supplyCurrentLimited = null;
                statorCurrentLimited = null;
                faultsEntry = null;
                stickyFaultsEntry = null;
                supplyCurrentLimitedEntry = null;
                statorCurrentLimitedEntry = null;
            }

            if (motorConfig.logLimits()) {
                forwardLimit = motor.getForwardLimit(false);
                reverseLimit = motor.getReverseLimit(false);
                slowSignals.addAll(List.of(forwardLimit, reverseLimit));
                forwardLimitEntry = new BooleanLogEntry(log, base + "/ForwardLimit");
                reverseLimitEntry = new BooleanLogEntry(log, base + "/ReverseLimit");
            } else {
                forwardLimit = null;
                reverseLimit = null;
                forwardLimitEntry = null;
                reverseLimitEntry = null;
            }

            controlMode = motor.getControlMode(false);
            version = motor.getVersion(false);
            slowSignals.addAll(List.of(controlMode, version));
            connectedEntry = new BooleanLogEntry(log, base + "/Connected");
            controlModeEntry = new StringLogEntry(log, base + "/ControlMode");
        }

        void writeMetadata() {
            String base = ROOT + "/Motors/" + subsystem + "/" + name;
            new IntegerLogEntry(log, base + "/DeviceId").append(motor.getDeviceID());
            new StringLogEntry(log, base + "/CANBus").append(motor.getNetwork().getName());
            new StringLogEntry(log, base + "/Role").append(motorConfig.role().toString());
            if (motorConfig.role() == MotorHealthConfig.Role.FOLLOWER) {
                new IntegerLogEntry(log, base + "/Follower/LeaderDeviceId")
                        .append(motorConfig.leaderDeviceId());
                new BooleanLogEntry(log, base + "/Follower/Opposed")
                        .append(motorConfig.followerOpposed());
            }
        }

        void logFast() {
            if (!connected) {
                return;
            }
            appendSignal(motorVoltageEntry, motorVoltage);
            appendSignal(velocityEntry, velocity);
            appendSignal(positionEntry, position);
            appendSignal(referenceEntry, reference);
            appendSignal(closedLoopErrorEntry, closedLoopError);
        }

        void logCurrents() {
            if (!connected) {
                lastSupplyCurrent = Double.NaN;
                lastStatorCurrent = Double.NaN;
                return;
            }
            lastSupplyCurrent = signalValue(supplyCurrent);
            lastStatorCurrent = signalValue(statorCurrent);
            supplyCurrentUpdatedThisCycle = isNewSignalSample(supplyCurrent);
            statorCurrentUpdatedThisCycle = isNewSignalSample(statorCurrent);
            if (supplyCurrentUpdatedThisCycle) {
                appendFinite(supplyCurrentEntry, lastSupplyCurrent);
            }
            if (statorCurrentUpdatedThisCycle) {
                appendFinite(statorCurrentEntry, lastStatorCurrent);
            }
            appendSignal(torqueCurrentEntry, torqueCurrent);
        }

        void logSlow() {
            connected = signalGood(version)
                    && version.getTimestamp().isValid()
                    && version.getTimestamp().getLatency() <= config.connectionTimeoutSeconds();
            update(connectedEntry, connected);
            if (!connected) {
                lastSupplyCurrent = Double.NaN;
                lastStatorCurrent = Double.NaN;
                return;
            }
            appendSignal(temperatureEntry, temperature);
            if (signalGood(faultField)) {
                update(faultsEntry, faultField.getValue());
            }
            if (signalGood(stickyFaultField)) {
                update(stickyFaultsEntry, stickyFaultField.getValue());
            }
            if (signalGood(supplyCurrentLimited)) {
                update(supplyCurrentLimitedEntry, supplyCurrentLimited.getValue());
            }
            if (signalGood(statorCurrentLimited)) {
                update(statorCurrentLimitedEntry, statorCurrentLimited.getValue());
            }
            if (signalGood(forwardLimit)) {
                update(
                        forwardLimitEntry,
                        forwardLimit.getValue() == ForwardLimitValue.ClosedToGround);
            }
            if (signalGood(reverseLimit)) {
                update(
                        reverseLimitEntry,
                        reverseLimit.getValue() == ReverseLimitValue.ClosedToGround);
            }
            if (signalGood(controlMode)) {
                update(controlModeEntry, controlMode.getValue().toString());
            }
        }
    }

    private final class SubsystemRegistration {
        final String name;
        final Subsystem subsystem;
        final BooleanSupplier activeSupplier;
        final Supplier<String> stateSupplier;
        final DoubleSupplier referenceSupplier;
        final DoubleSupplier measuredSupplier;
        final int[] pdhChannels;
        final BooleanLogEntry activeEntry;
        final StringLogEntry stateEntry;
        final StringLogEntry currentCommandEntry;
        final DoubleLogEntry supplyCurrentEntry;
        final DoubleLogEntry statorCurrentEntry;
        final DoubleLogEntry energyEntry;
        final DoubleLogEntry referenceEntry;
        final DoubleLogEntry measuredEntry;
        final DoubleLogEntry errorEntry;
        final StringLogEntry currentSourceEntry;
        double energyWh;
        double lastNumericTimestamp = Double.NaN;

        SubsystemRegistration(
                String name,
                Subsystem subsystem,
                BooleanSupplier activeSupplier,
                Supplier<String> stateSupplier,
                DoubleSupplier referenceSupplier,
                DoubleSupplier measuredSupplier,
                int[] pdhChannels) {
            this.name = name;
            this.subsystem = subsystem;
            this.activeSupplier = activeSupplier;
            this.stateSupplier = stateSupplier;
            this.referenceSupplier = referenceSupplier;
            this.measuredSupplier = measuredSupplier;
            this.pdhChannels = Arrays.stream(pdhChannels)
                    .filter(channel -> channel >= 0)
                    .distinct()
                    .toArray();
            String base = ROOT + "/Subsystems/" + name;
            activeEntry = new BooleanLogEntry(log, base + "/Active");
            stateEntry = new StringLogEntry(log, base + "/State");
            currentCommandEntry = new StringLogEntry(log, base + "/CurrentCommand");
            supplyCurrentEntry = new DoubleLogEntry(log, base + "/SupplyCurrent");
            statorCurrentEntry = new DoubleLogEntry(log, base + "/StatorCurrent");
            energyEntry = new DoubleLogEntry(log, base + "/EnergyWh");
            referenceEntry = new DoubleLogEntry(log, base + "/Reference");
            measuredEntry = new DoubleLogEntry(log, base + "/Measured");
            errorEntry = new DoubleLogEntry(log, base + "/ClosedLoopError");
            currentSourceEntry = new StringLogEntry(log, base + "/CurrentSource");
        }

        void logState() {
            try {
                update(activeEntry, activeSupplier.getAsBoolean());
            } catch (RuntimeException | LinkageError ex) {
                noteFailure("subsystem-active-" + name, ex);
            }
            try {
                update(stateEntry, stateSupplier.get());
            } catch (RuntimeException | LinkageError ex) {
                noteFailure("subsystem-state-" + name, ex);
            }
            try {
                Command currentCommand = subsystem == null ? null : subsystem.getCurrentCommand();
                update(
                        currentCommandEntry,
                        currentCommand == null ? "None" : currentCommand.getName());
            } catch (RuntimeException | LinkageError ex) {
                noteFailure("subsystem-command-" + name, ex);
            }
        }

        void resetSessionEnergy() {
            energyWh = 0.0;
            lastNumericTimestamp = Double.NaN;
            appendFinite(energyEntry, energyWh);
        }

        void logNumeric(double now) {
            double elapsedSeconds = Double.isFinite(lastNumericTimestamp)
                    ? Math.max(0.0, now - lastNumericTimestamp)
                    : 0.0;
            lastNumericTimestamp = now;
            double supply = subsystemSupplyCurrent();
            double stator = motorCurrentSum(name, false);
            if (supplyCurrentUpdated()) {
                appendFinite(supplyCurrentEntry, supply);
            }
            if (motorCurrentUpdated(name, false)) {
                appendFinite(statorCurrentEntry, stator);
            }
            update(currentSourceEntry, supplyCurrentSource());
            if (Double.isFinite(supply) && elapsedSeconds > 0.0) {
                double voltage = RobotController.getBatteryVoltage();
                if (Double.isFinite(voltage)) {
                    energyWh += voltage * supply * elapsedSeconds / 3600.0;
                    appendFinite(energyEntry, energyWh);
                }
            }

            double reference = supplierValue(referenceSupplier);
            double measured = supplierValue(measuredSupplier);
            appendFinite(referenceEntry, reference);
            appendFinite(measuredEntry, measured);
            if (Double.isFinite(reference) && Double.isFinite(measured)) {
                appendFinite(errorEntry, reference - measured);
            }
        }

        String supplyCurrentSource() {
            return powerDistribution != null && validPdhChannelCount() > 0
                    ? "PDHChannels"
                    : hasMotorForSubsystem(name) ? "TalonFXSupplyCurrent" : "Unavailable";
        }

        double subsystemSupplyCurrent() {
            if (powerDistribution != null && validPdhChannelCount() > 0) {
                double sum = 0.0;
                int count = 0;
                int expected = pdhChannels.length;
                for (int channel : pdhChannels) {
                    if (channel < pdhHardwareChannelCount
                            && channel < pdhCurrents.length
                            && Double.isFinite(pdhCurrents[channel])) {
                        sum += pdhCurrents[channel];
                        count++;
                    }
                }
                return count == expected ? sum : Double.NaN;
            }
            return motorCurrentSum(name, true);
        }

        boolean supplyCurrentUpdated() {
            return powerDistribution != null && validPdhChannelCount() > 0
                    ? pdhUpdatedThisCycle
                    : motorCurrentUpdated(name, true);
        }

        int validPdhChannelCount() {
            int count = 0;
            for (int channel : pdhChannels) {
                if (channel < pdhHardwareChannelCount) {
                    count++;
                }
            }
            return count;
        }
    }

    private boolean hasMotorForSubsystem(String subsystem) {
        for (MotorRegistration motor : motors.values()) {
            if (motor.subsystem.equals(subsystem)) {
                return true;
            }
        }
        return false;
    }

    private double motorCurrentSum(String subsystem, boolean supply) {
        double sum = 0.0;
        int count = 0;
        for (MotorRegistration motor : motors.values()) {
            if (!motor.subsystem.equals(subsystem)) {
                continue;
            }
            double value = supply ? motor.lastSupplyCurrent : motor.lastStatorCurrent;
            if (!Double.isFinite(value)) {
                // A partial motor sum is not a subsystem total.
                return Double.NaN;
            }
            sum += value;
            count++;
        }
        return count > 0 ? sum : Double.NaN;
    }

    private boolean motorCurrentUpdated(String subsystem, boolean supply) {
        for (MotorRegistration motor : motors.values()) {
            if (motor.subsystem.equals(subsystem)
                    && (supply
                            ? motor.supplyCurrentUpdatedThisCycle
                            : motor.statorCurrentUpdatedThisCycle)) {
                return true;
            }
        }
        return false;
    }

    private static double supplierValue(DoubleSupplier supplier) {
        if (supplier == null) {
            return Double.NaN;
        }
        try {
            return supplier.getAsDouble();
        } catch (RuntimeException | LinkageError ex) {
            return Double.NaN;
        }
    }

    private static double signalValue(BaseStatusSignal signal) {
        return signalGood(signal) ? signal.getValueAsDouble() : Double.NaN;
    }

    private boolean isNewSignalSample(BaseStatusSignal signal) {
        // Phoenix implements hasUpdated() against its guaranteed-monotonic system
        // timestamp. This also recovers if a replay/source clock is restarted.
        return signalGood(signal) && signal.hasUpdated();
    }

    private void appendSignal(DoubleLogEntry entry, BaseStatusSignal signal) {
        if (isNewSignalSample(signal)) {
            appendFinite(entry, signalValue(signal));
        }
    }
}
