package frc.robot.health.analyzer.rules;

/** Health conditions understood by the configurable rule engine. */
public enum RuleType {
    LOW_VOLTAGE,
    BROWNOUT,
    HIGH_TOTAL_CURRENT,
    STALL,
    MOTOR_IMBALANCE,
    FOLLOWER_FAULT,
    TRACKING_ERROR,
    TEMPERATURE,
    CAN_FAULT,
    IDLE_DRAW
}
