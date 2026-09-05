package frc.robot.config;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants;

/** The same joystick response in physical units on either robot; no heading input is needed. */
public final class DriverControls {
    private DriverControls() {}

    /** Inputs are forward, left, and counter-clockwise, each normalized to -1 through +1. */
    public static ChassisSpeeds toRobotSpeeds(double forward, double left, double turn) {
        double magnitude = Math.hypot(forward, left);
        double translation = MathUtil.applyDeadband(
                Math.min(magnitude, 1.0), Constants.OperatorConstants.TRANSLATION_DEADBAND)
                * Constants.DriveConstants.COMMON_SPEED_METERS_PER_SECOND
                * Constants.OperatorConstants.DRIVE_SPEED_SCALE;
        double scale = magnitude > 0.0 ? translation / magnitude : 0.0;
        double rotation = MathUtil.applyDeadband(
                MathUtil.clamp(turn, -1.0, 1.0), Constants.OperatorConstants.ROTATION_DEADBAND)
                * Constants.DriveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND
                * Constants.OperatorConstants.TURN_SPEED_SCALE;
        return new ChassisSpeeds(forward * scale, left * scale, rotation);
    }
}
