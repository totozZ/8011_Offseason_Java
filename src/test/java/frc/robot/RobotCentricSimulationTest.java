package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import frc.robot.config.DrivetrainProfile;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.junit.jupiter.api.Test;

/** Runs separately from hardware-constant tests because CTRE mutates configuration objects. */
abstract class RobotCentricSimulationTest {
    protected abstract DrivetrainProfile profile();

    @Test
    void robotRelativeTargetsStillUpdateAfterPigeonIsRemoved() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Unmanaged.feedEnable(5000);
        try {
            try (var drivetrain = profile().createDrivetrain()) {
                // Let simulation enable and its CAN devices finish starting before the first request.
                pumpDriverStation(500);
                var request = CommandSwerveDrivetrain.createRobotCentricRequest();
                drivetrain.resetPose(new Pose2d(2.0, 3.0, Rotation2d.kCW_90deg));
                drivetrain.setOperatorPerspectiveForward(Rotation2d.k180deg);
                drivetrain.setControl(request.withVelocityX(0.6));
                pumpDriverStation(250);
                assertForwardTargets(drivetrain);

                // Remove only the simulated Pigeon device. Native swerve and all modules remain alive.
                // Pin this test-only JNI use to the repository's Phoenix 26.3.0 dependency.
                var yaw = drivetrain.getPigeon2().getYaw();
                assertEquals(0, PlatformJNI.JNI_SimDestroy(
                        DeviceType.P6_Pigeon2Type.value, drivetrain.getPigeon2().getDeviceID()));
                pumpDriverStation(700);
                assertFalse(yaw.refresh().getStatus().isOK(), "Pigeon yaw must actually be stale");
                drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.4));
                pumpDriverStation(150);
                assertChassisTarget(drivetrain, 0.0, 0.4, 0.0);
                drivetrain.setControl(request.withVelocityY(0.0).withRotationalRate(1.0));
                pumpDriverStation(150);
                assertChassisTarget(drivetrain, 0.0, 0.0, 1.0);
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
                pumpDriverStation(100);
                assertChassisTarget(drivetrain, 0.0, 0.0, 0.0);
                drivetrain.setControl(CommandSwerveDrivetrain.createRobotCentricRequest());
                pumpDriverStation(100);
                for (var target : drivetrain.getStateCopy().ModuleTargets) {
                    assertEquals(0.0, target.speedMetersPerSecond, 1e-6);
                }
            }
        } finally {
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
            Unmanaged.feedEnable(0);
        }
    }

    private static void pumpDriverStation(int milliseconds) throws InterruptedException {
        for (int elapsed = 0; elapsed < milliseconds; elapsed += 20) {
            DriverStationSim.notifyNewData();
            Unmanaged.feedEnable(100);
            Thread.sleep(20);
        }
    }

    private static void assertForwardTargets(CommandSwerveDrivetrain drivetrain) {
        assertChassisTarget(drivetrain, 0.6, 0.0, 0.0);
    }

    private static void assertChassisTarget(
            CommandSwerveDrivetrain drivetrain, double vx, double vy, double omega) {
        // CTRE may reverse wheel speed and add 180 degrees during module optimization.
        // Reconstructing chassis velocity checks the resulting motion, independent of that choice.
        var speeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getStateCopy().ModuleTargets);
        assertEquals(vx, speeds.vxMetersPerSecond, 1e-6,
                () -> "Targets=" + java.util.Arrays.toString(drivetrain.getStateCopy().ModuleTargets)
                        + ", DAQs=" + drivetrain.getStateCopy().SuccessfulDaqs
                        + ", failed=" + drivetrain.getStateCopy().FailedDaqs);
        assertEquals(vy, speeds.vyMetersPerSecond, 1e-6);
        assertEquals(omega, speeds.omegaRadiansPerSecond, 1e-6);
    }
}

class NormalRobotCentricSimulationTest extends RobotCentricSimulationTest {
    @Override
    protected DrivetrainProfile profile() {
        return DrivetrainProfile.NORMAL;
    }
}

class SoccerBotRobotCentricSimulationTest extends RobotCentricSimulationTest {
    @Override
    protected DrivetrainProfile profile() {
        return DrivetrainProfile.SOCCER_BOT;
    }
}
