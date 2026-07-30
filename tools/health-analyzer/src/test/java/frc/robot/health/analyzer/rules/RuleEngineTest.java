package frc.robot.health.analyzer.rules;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import frc.robot.health.analyzer.model.HealthLog;
import frc.robot.health.analyzer.model.Sample;
import java.io.StringReader;
import java.time.Duration;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class RuleEngineTest {
    @Test
    void detectsAllTenRuleFamilies() {
        HealthLog log = fixture();
        HealthRules rules = HealthRules.fromJson(new StringReader("""
                {
                  "defaults": {
                    "LOW_VOLTAGE":{"durationSeconds":0,"thresholds":{"voltage":7}},
                    "BROWNOUT":{"durationSeconds":0},
                    "HIGH_TOTAL_CURRENT":{"durationSeconds":0,"thresholds":{"rawCurrent":200,"rollingCurrent":150,"rollingWindowMs":100}},
                    "STALL":{"durationSeconds":0,"thresholds":{"minOutputVolts":2,"minStatorCurrent":50,"maxAbsVelocity":1}},
                    "MOTOR_IMBALANCE":{"durationSeconds":0,"thresholds":{"ratio":0.3,"minCurrent":5}},
                    "FOLLOWER_FAULT":{"durationSeconds":0,"thresholds":{"leaderMinCurrent":10,"followerMaxCurrent":2,"followerMaxAbsVelocity":1}},
                    "TRACKING_ERROR":{"durationSeconds":0,"thresholds":{"error":5}},
                    "TEMPERATURE":{"durationSeconds":0,"thresholds":{"temperature":80,"riseCPerSecond":5}},
                    "CAN_FAULT":{"durationSeconds":0,"thresholds":{"utilization":0.8}},
                    "IDLE_DRAW":{"durationSeconds":0,"thresholds":{"supplyCurrent":8,"statorCurrent":10,"motorVoltage":1}}
                  },
                  "followers":[{"subsystem":"Drive","leader":"Leader","follower":"Follower"}]
                }
                """));

        EnumSet<RuleType> detected = new RuleEngine().evaluate(log, rules).stream()
                .map(Anomaly::rule)
                .collect(Collectors.toCollection(() -> EnumSet.noneOf(RuleType.class)));

        assertEquals(EnumSet.allOf(RuleType.class), detected);
    }

    @Test
    void doesNotReportMotorImbalanceWhileSubsystemIsInactive() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Drive/Active",
                                List.of(bool(0, false), bool(2_000_000, false))),
                        Map.of(
                                "/Health/Motors/Drive/Left/SupplyCurrent",
                                doubles(40, 40, 40),
                                "/Health/Motors/Drive/Right/SupplyCurrent",
                                doubles(5, 5, 5)),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "MOTOR_IMBALANCE": {
                                      "durationSeconds": 0,
                                      "thresholds": {"ratio": 0.3, "minCurrent": 5}
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.MOTOR_IMBALANCE)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void comparesOnlyDeclaredFollowerGroupUsingCurrentMagnitudes() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Shooter/Active",
                                List.of(bool(0, true), bool(2_000_000, true))),
                        Map.of(
                                "/Health/Motors/Shooter/Leader/SupplyCurrent",
                                doubles(-20, -20, -20),
                                "/Health/Motors/Shooter/Follower/SupplyCurrent",
                                doubles(20, 20, 20),
                                "/Health/Motors/Shooter/Pitch/SupplyCurrent",
                                doubles(0, 0, 0)),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "MOTOR_IMBALANCE": {
                                      "durationSeconds": 0,
                                      "thresholds": {"ratio": 0.3, "minCurrent": 5}
                                    }
                                  },
                                  "followers": [
                                    {
                                      "subsystem": "Shooter",
                                      "leader": "Leader",
                                      "follower": "Follower"
                                    }
                                  ]
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.MOTOR_IMBALANCE)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void doesNotCompareIndependentMotorsWithDifferentJobs() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Feeder/Active",
                                List.of(bool(0, true), bool(2_000_000, true))),
                        Map.of(
                                "/Health/Motors/Feeder/Backward/SupplyCurrent",
                                doubles(40, 40, 40),
                                "/Health/Motors/Feeder/Upward/SupplyCurrent",
                                doubles(5, 5, 5)),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "MOTOR_IMBALANCE": {
                                      "durationSeconds": 0,
                                      "thresholds": {"ratio": 0.3, "minCurrent": 5}
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.MOTOR_IMBALANCE)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void detectsBrownoutFromAnEventWhenTheBooleanSignalIsUnavailable() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(),
                        Map.of(
                                HealthLog.EVENT_CATEGORY_PATH,
                                List.of(text(1_000_000, "Brownout")),
                                HealthLog.EVENT_MESSAGE_PATH,
                                List.of(text(1_000_000, "Controller event"))),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "BROWNOUT": {"durationSeconds": 0}
                                  }
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.BROWNOUT)
                        .toList();

        assertEquals(1, findings.size());
        assertEquals(1_000_000, findings.get(0).startMicros());
    }

    @Test
    void doesNotExtendExpiredNumericTelemetryToTheEndOfTheLog() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/TotalCurrent",
                                List.of(
                                        number(0, 300),
                                        number(100_000, 300))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "HIGH_TOTAL_CURRENT": {
                                      "durationSeconds": 0.5,
                                      "thresholds": {
                                        "rawCurrent": 200,
                                        "rollingCurrent": 200,
                                        "rollingWindowMs": 100
                                      }
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(
                                anomaly ->
                                        anomaly.rule()
                                                == RuleType.HIGH_TOTAL_CURRENT)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void derivesFollowerMetadataAndDetectsDisconnectedStaleFollower() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Motors/Shooter/Leader/Connected",
                                List.of(bool(0, true)),
                                "/Health/Motors/Shooter/Follower/Connected",
                                List.of(bool(0, true), bool(1_000_000, false))),
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Motors/Shooter/Leader/DeviceId",
                                        List.of(number(0, 10))),
                                Map.entry(
                                        "/Health/Motors/Shooter/Leader/SupplyCurrent",
                                        doubles(20, 20, 20)),
                                Map.entry(
                                        "/Health/Motors/Shooter/Leader/MotorVoltage",
                                        doubles(6, 6, 6)),
                                Map.entry(
                                        "/Health/Motors/Shooter/Follower/DeviceId",
                                        List.of(number(0, 11))),
                                Map.entry(
                                        "/Health/Motors/Shooter/Follower/Follower/LeaderDeviceId",
                                        List.of(number(0, 10))),
                                Map.entry(
                                        "/Health/Motors/Shooter/Follower/SupplyCurrent",
                                        List.of(number(0, 18), number(500_000, 18))),
                                Map.entry(
                                        "/Health/Motors/Shooter/Follower/Velocity",
                                        List.of(number(0, 10), number(500_000, 10)))),
                        Map.of(
                                "/Health/Motors/Shooter/Leader/Role",
                                List.of(text(0, "LEADER")),
                                "/Health/Motors/Shooter/Follower/Role",
                                List.of(text(0, "FOLLOWER"))),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 0.4,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "leaderMinOutputVolts": 1,
                                        "maxSampleAgeSeconds": 0.75
                                      }
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.FOLLOWER_FAULT)
                        .count();

        assertEquals(1, findings);
    }

    @Test
    void derivesFollowerLeaderOnTheSameCanBusAndTreatsEitherWeakSignalAsFault() {
        Map<String, List<Sample<Double>>> doubles = new LinkedHashMap<>();
        doubles.put(
                "/Health/Motors/Mechanism/CanLeader/DeviceId",
                List.of(number(0, 10)));
        doubles.put(
                "/Health/Motors/Mechanism/CanLeader/SupplyCurrent",
                doubles(20, 20, 20));
        doubles.put(
                "/Health/Motors/Mechanism/CanLeader/MotorVoltage",
                doubles(6, 6, 6));
        doubles.put(
                "/Health/Motors/Mechanism/RioDuplicate/DeviceId",
                List.of(number(0, 10)));
        doubles.put(
                "/Health/Motors/Mechanism/RioDuplicate/SupplyCurrent",
                doubles(0, 0, 0));
        doubles.put(
                "/Health/Motors/Mechanism/RioDuplicate/MotorVoltage",
                doubles(0, 0, 0));
        doubles.put(
                "/Health/Motors/Mechanism/Follower/DeviceId",
                List.of(number(0, 11)));
        doubles.put(
                "/Health/Motors/Mechanism/Follower/Follower/LeaderDeviceId",
                List.of(number(0, 10)));
        doubles.put(
                "/Health/Motors/Mechanism/Follower/SupplyCurrent",
                doubles(1, 1, 1));
        doubles.put(
                "/Health/Motors/Mechanism/Follower/Velocity",
                doubles(15, 15, 15));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        doubles,
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Motors/Mechanism/CanLeader/Role",
                                        List.of(text(0, "LEADER"))),
                                Map.entry(
                                        "/Health/Motors/Mechanism/CanLeader/CANBus",
                                        List.of(text(0, "CANivore"))),
                                Map.entry(
                                        "/Health/Motors/Mechanism/RioDuplicate/Role",
                                        List.of(text(0, "INDEPENDENT"))),
                                Map.entry(
                                        "/Health/Motors/Mechanism/RioDuplicate/CANBus",
                                        List.of(text(0, "rio"))),
                                Map.entry(
                                        "/Health/Motors/Mechanism/Follower/Role",
                                        List.of(text(0, "FOLLOWER"))),
                                Map.entry(
                                        "/Health/Motors/Mechanism/Follower/CANBus",
                                        List.of(text(0, "CANivore")))),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 0.1,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "leaderMinOutputVolts": 1,
                                        "followerMaxCurrent": 2,
                                        "followerMaxAbsVelocity": 1
                                      }
                                    }
                                  }
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.FOLLOWER_FAULT)
                        .toList();

        assertEquals(1, findings.size());
        assertEquals("Follower", findings.get(0).motor());
    }

    @Test
    void skipsMotorImbalanceWhenAnyRegisteredMotorLacksFullCurrentCoverage() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Drive/Active",
                                List.of(bool(0, true), bool(2_000_000, true))),
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Motors/Drive/Left/SupplyCurrent",
                                        doubles(40, 40, 40)),
                                Map.entry(
                                        "/Health/Motors/Drive/Middle/SupplyCurrent",
                                        doubles(5, 5, 5)),
                                Map.entry(
                                        "/Health/Motors/Drive/Right/SupplyCurrent",
                                        List.of(
                                                number(0, 20),
                                                number(500_000, 20)))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "MOTOR_IMBALANCE": {
                                      "durationSeconds": 0,
                                      "thresholds": {"ratio": 0.3, "minCurrent": 5}
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.MOTOR_IMBALANCE)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void evaluatesSubsystemTrackingEvenWhenAMotorHasItsOwnErrorSignal() {
        HealthLog log =
                new HealthLog(
                        Map.of(
                                "/Health/Subsystems/Arm/Active",
                                List.of(bool(0, true), bool(2_000_000, true))),
                        Map.ofEntries(
                                Map.entry(
                                        "/Health/Subsystems/Arm/Reference",
                                        doubles(10, 10, 10)),
                                Map.entry(
                                        "/Health/Subsystems/Arm/Measured",
                                        doubles(0, 0, 0)),
                                Map.entry(
                                        "/Health/Motors/Arm/Main/ClosedLoopError",
                                        doubles(10, 10, 10))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "TRACKING_ERROR": {
                                      "durationSeconds": 0,
                                      "thresholds": {"error": 5}
                                    }
                                  }
                                }
                                """));

        long subsystemFindings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(
                                anomaly ->
                                        anomaly.rule() == RuleType.TRACKING_ERROR
                                                && anomaly.subsystem().equals("Arm")
                                                && anomaly.motor().isBlank())
                        .count();

        assertEquals(1, subsystemFindings);
    }

    @Test
    void usesTheRealFinalTemperatureSegmentForRiseDuration() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Arm/Main/Temperature",
                                doubles(20, 20, 30)),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "TEMPERATURE": {
                                      "durationSeconds": 0.75,
                                      "thresholds": {
                                        "temperature": 100,
                                        "riseCPerSecond": 5
                                      }
                                    }
                                  }
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.TEMPERATURE)
                        .toList();

        assertEquals(1, findings.size());
        assertEquals(1_000_000L, findings.get(0).startMicros());
        assertEquals(2_000_000L, findings.get(0).endMicros());
    }

    @Test
    void doesNotExtendTemperatureRiseAcrossATelemetryGap() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Arm/Main/Temperature",
                                List.of(
                                        number(0, 20),
                                        number(1_000_000, 30),
                                        number(3_000_000, 30))),
                        Map.of(),
                        0,
                        3_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "TEMPERATURE": {
                                      "durationSeconds": 1.5,
                                      "thresholds": {
                                        "temperature": 100,
                                        "riseCPerSecond": 5
                                      }
                                    }
                                  }
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.TEMPERATURE)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void detectsIndependentStallIntervalsOnBothSidesOfATelemetryGap() {
        List<Sample<Double>> highLoad =
                List.of(
                        number(0, 80),
                        number(500_000, 80),
                        number(1_000_000, 80),
                        number(3_000_000, 80),
                        number(3_500_000, 80),
                        number(4_000_000, 80));
        List<Sample<Double>> stopped =
                List.of(
                        number(0, 0),
                        number(500_000, 0),
                        number(1_000_000, 0),
                        number(3_000_000, 0),
                        number(3_500_000, 0),
                        number(4_000_000, 0));
        List<Sample<Double>> commanded =
                List.of(
                        number(0, 6),
                        number(500_000, 6),
                        number(1_000_000, 6),
                        number(3_000_000, 6),
                        number(3_500_000, 6),
                        number(4_000_000, 6));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Arm/Main/StatorCurrent", highLoad,
                                "/Health/Motors/Arm/Main/Velocity", stopped,
                                "/Health/Motors/Arm/Main/MotorVoltage", commanded),
                        Map.of(),
                        0,
                        4_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "STALL": {
                                      "durationSeconds": 0.75,
                                      "thresholds": {
                                        "minOutputVolts": 2,
                                        "minStatorCurrent": 50,
                                        "maxAbsVelocity": 1
                                      }
                                    }
                                  }
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.STALL)
                        .toList();

        assertEquals(2, findings.size());
        assertEquals(0L, findings.get(0).startMicros());
        assertEquals(1_000_000L, findings.get(0).endMicros());
        assertEquals(3_000_000L, findings.get(1).startMicros());
        assertEquals(4_000_000L, findings.get(1).endMicros());
    }

    @Test
    void detectsRollingHighCurrentIntervalsOnBothSidesOfATelemetryGap() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Power/TotalCurrent",
                                List.of(
                                        number(0, 300),
                                        number(500_000, 300),
                                        number(1_000_000, 300),
                                        number(3_000_000, 300),
                                        number(3_500_000, 300),
                                        number(4_000_000, 300))),
                        Map.of(),
                        0,
                        4_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "HIGH_TOTAL_CURRENT": {
                                      "durationSeconds": 0.75,
                                      "thresholds": {
                                        "rawCurrent": 1000,
                                        "rollingCurrent": 200,
                                        "rollingWindowMs": 100
                                      }
                                    }
                                  }
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.HIGH_TOTAL_CURRENT)
                        .toList();

        assertEquals(2, findings.size());
        assertEquals(0L, findings.get(0).startMicros());
        assertEquals(1_000_000L, findings.get(0).endMicros());
        assertEquals(3_000_000L, findings.get(1).startMicros());
        assertEquals(4_000_000L, findings.get(1).endMicros());
    }

    @Test
    void doesNotHoldAWeakFollowerFaultPastTheFreshnessBoundary() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Leader/SupplyCurrent",
                                List.of(number(0, 20), number(2_000_000, 0)),
                                "/Health/Motors/Drive/Follower/SupplyCurrent",
                                List.of(number(0, 1), number(2_000_000, 20))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 1,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "followerMaxCurrent": 2,
                                        "maxSampleAgeSeconds": 0.75
                                      }
                                    }
                                  },
                                  "followers": [
                                    {
                                      "subsystem": "Drive",
                                      "leader": "Leader",
                                      "follower": "Follower"
                                    }
                                  ]
                                }
                                """));

        long findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.FOLLOWER_FAULT)
                        .count();

        assertEquals(0, findings);
    }

    @Test
    void startsFollowerStalenessOneMicrosecondAfterTheInclusiveFreshnessLimit() {
        List<Sample<Double>> loadedLeader =
                List.of(
                        number(0, 20),
                        number(500_000, 20),
                        number(1_000_000, 20),
                        number(1_500_000, 20),
                        number(2_000_000, 20));
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Leader/SupplyCurrent",
                                loadedLeader,
                                "/Health/Motors/Drive/Follower/SupplyCurrent",
                                List.of(number(0, 20), number(2_000_000, 20))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 1.1,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "followerMaxCurrent": 2,
                                        "maxSampleAgeSeconds": 0.75
                                      }
                                    }
                                  },
                                  "followers": [
                                    {
                                      "subsystem": "Drive",
                                      "leader": "Leader",
                                      "follower": "Follower"
                                    }
                                  ]
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.FOLLOWER_FAULT)
                        .toList();

        assertEquals(1, findings.size());
        assertEquals(750_001L, findings.get(0).startMicros());
        assertEquals(2_000_000L, findings.get(0).endMicros());
    }

    @Test
    void treatsFollowerTelemetryAsUnknownBeforeItsFirstSample() {
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Leader/SupplyCurrent",
                                doubles(20, 20, 20),
                                "/Health/Motors/Drive/Follower/SupplyCurrent",
                                List.of(number(1_000_000, 1), number(2_000_000, 1))),
                        Map.of(),
                        0,
                        2_000_000);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 0,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "followerMaxCurrent": 2,
                                        "maxSampleAgeSeconds": 10
                                      }
                                    }
                                  },
                                  "followers": [
                                    {
                                      "subsystem": "Drive",
                                      "leader": "Leader",
                                      "follower": "Follower"
                                    }
                                  ]
                                }
                                """));

        List<Anomaly> findings =
                new RuleEngine().evaluate(log, rules).stream()
                        .filter(anomaly -> anomaly.rule() == RuleType.FOLLOWER_FAULT)
                        .toList();

        assertEquals(1, findings.size());
        assertEquals(1_000_000L, findings.get(0).startMicros());
    }

    @Test
    void keepsDenseFollowerStalenessEvaluationWithinAReasonableBound() {
        int sampleCount = 20_001;
        List<Sample<Double>> leader = denseSamples(sampleCount, 20.0);
        List<Sample<Double>> follower = denseSamples(sampleCount, 20.0);
        long endMicros = leader.get(leader.size() - 1).timestampMicros();
        HealthLog log =
                new HealthLog(
                        Map.of(),
                        Map.of(
                                "/Health/Motors/Drive/Leader/SupplyCurrent",
                                leader,
                                "/Health/Motors/Drive/Follower/SupplyCurrent",
                                follower),
                        Map.of(),
                        0,
                        endMicros);
        HealthRules rules =
                HealthRules.fromJson(
                        new StringReader(
                                """
                                {
                                  "defaults": {
                                    "FOLLOWER_FAULT": {
                                      "durationSeconds": 0,
                                      "thresholds": {
                                        "leaderMinCurrent": 10,
                                        "followerMaxCurrent": 2,
                                        "maxSampleAgeSeconds": 1
                                      }
                                    }
                                  },
                                  "followers": [
                                    {
                                      "subsystem": "Drive",
                                      "leader": "Leader",
                                      "follower": "Follower"
                                    }
                                  ]
                                }
                                """));

        assertTimeoutPreemptively(
                Duration.ofSeconds(5),
                () -> new RuleEngine().evaluate(log, rules));
    }

    private static HealthLog fixture() {
        Map<String, List<Sample<Boolean>>> booleans = Map.of(
                "/Health/Robot/BrownedOut",
                List.of(bool(0, false), bool(1_000_000, true), bool(2_000_000, false)),
                "/Health/Subsystems/Drive/Active",
                List.of(bool(0, true), bool(2_000_000, true)));
        Map<String, List<Sample<String>>> strings = Map.of(
                "/Health/Subsystems/Drive/State",
                List.of(text(0, "Idle"), text(2_000_000, "Idle")));
        Map<String, List<Sample<Double>>> doubles = Map.ofEntries(
                Map.entry("/Health/Power/Voltage", doubles(6, 6, 6)),
                Map.entry("/Health/Power/TotalCurrent", doubles(300, 300, 300)),
                Map.entry("/Health/Robot/CAN/Utilization", doubles(.9, .9, .9)),
                Map.entry("/Health/Robot/CAN/BusOffCount", doubles(0, 1, 1)),
                Map.entry("/Health/Motors/Drive/Leader/SupplyCurrent", doubles(20, 20, 20)),
                Map.entry("/Health/Motors/Drive/Leader/StatorCurrent", doubles(80, 80, 80)),
                Map.entry("/Health/Motors/Drive/Leader/MotorVoltage", doubles(6, 6, 6)),
                Map.entry("/Health/Motors/Drive/Leader/Velocity", doubles(0, 0, 0)),
                Map.entry("/Health/Motors/Drive/Leader/ClosedLoopError", doubles(10, 10, 10)),
                Map.entry("/Health/Motors/Drive/Leader/Temperature", doubles(20, 30, 90)),
                Map.entry("/Health/Motors/Drive/Follower/SupplyCurrent", doubles(1, 1, 1)),
                Map.entry("/Health/Motors/Drive/Follower/StatorCurrent", doubles(12, 12, 12)),
                Map.entry("/Health/Motors/Drive/Follower/MotorVoltage", doubles(2, 2, 2)),
                Map.entry("/Health/Motors/Drive/Follower/Velocity", doubles(0, 0, 0)),
                Map.entry("/Health/Motors/Drive/Follower/Temperature", doubles(20, 20, 20)));
        return new HealthLog(booleans, doubles, strings, 0, 2_000_000);
    }

    private static List<Sample<Double>> doubles(double a, double b, double c) {
        return List.of(number(0, a), number(1_000_000, b), number(2_000_000, c));
    }

    private static List<Sample<Double>> denseSamples(int count, double value) {
        List<Sample<Double>> result = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            result.add(number(i * 1_000L, value));
        }
        return List.copyOf(result);
    }

    private static Sample<Double> number(long timestamp, double value) {
        return new Sample<>(timestamp, value);
    }

    private static Sample<Boolean> bool(long timestamp, boolean value) {
        return new Sample<>(timestamp, value);
    }

    private static Sample<String> text(long timestamp, String value) {
        return new Sample<>(timestamp, value);
    }
}
