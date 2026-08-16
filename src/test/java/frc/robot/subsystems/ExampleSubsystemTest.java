package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

import org.junit.jupiter.api.Test;

class ExampleSubsystemTest {
    private static final double EPSILON = 1e-9;

    @Test
    void exampleHardwareIsDisabledByDefault() {
        assertFalse(Constants.ExampleConstants.ENABLE_EXAMPLE_SUBSYSTEM);
    }

    @Test
    void configurationDemonstratesRequiredSafetyAndClosedLoopFields() {
        TalonFXConfiguration configuration = ExampleSubsystem.createMotorConfiguration();

        assertEquals(NeutralModeValue.Brake, configuration.MotorOutput.NeutralMode);
        assertEquals(0.50, configuration.MotorOutput.PeakForwardDutyCycle, EPSILON);
        assertEquals(-0.50, configuration.MotorOutput.PeakReverseDutyCycle, EPSILON);

        assertEquals(1.0, configuration.Slot0.kP, EPSILON);
        assertEquals(0.0, configuration.Slot0.kI, EPSILON);
        assertEquals(0.0, configuration.Slot0.kD, EPSILON);
        assertEquals(0.20, configuration.Slot0.kS, EPSILON);
        assertEquals(0.12, configuration.Slot0.kV, EPSILON);
        assertEquals(0.01, configuration.Slot0.kA, EPSILON);

        assertEquals(10.0, configuration.MotionMagic.MotionMagicCruiseVelocity, EPSILON);
        assertEquals(20.0, configuration.MotionMagic.MotionMagicAcceleration, EPSILON);
        assertEquals(200.0, configuration.MotionMagic.MotionMagicJerk, EPSILON);
        assertEquals(10.0, configuration.Feedback.SensorToMechanismRatio, EPSILON);

        assertTrue(configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable);
        assertTrue(configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable);
        assertEquals(5.0, configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold, EPSILON);
        assertEquals(-5.0, configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold, EPSILON);

        assertTrue(configuration.CurrentLimits.SupplyCurrentLimitEnable);
        assertTrue(configuration.CurrentLimits.StatorCurrentLimitEnable);
        assertEquals(30.0, configuration.CurrentLimits.SupplyCurrentLimit, EPSILON);
        assertEquals(60.0, configuration.CurrentLimits.StatorCurrentLimit, EPSILON);
        assertEquals(40.0, configuration.TorqueCurrent.PeakForwardTorqueCurrent, EPSILON);
        assertEquals(-40.0, configuration.TorqueCurrent.PeakReverseTorqueCurrent, EPSILON);
    }

    @Test
    void reusableRequestObjectsAndNeutralModeMethodsArePresent() throws Exception {
        Set<Class<?>> requestTypes = Arrays.stream(ExampleSubsystem.class.getDeclaredFields())
                .map(field -> field.getType())
                .collect(Collectors.toSet());

        assertTrue(requestTypes.contains(NeutralOut.class));
        assertTrue(requestTypes.contains(DutyCycleOut.class));
        assertTrue(requestTypes.contains(VoltageOut.class));
        assertTrue(requestTypes.contains(VelocityVoltage.class));
        assertTrue(requestTypes.contains(PositionVoltage.class));
        assertTrue(requestTypes.contains(MotionMagicVoltage.class));
        assertTrue(requestTypes.contains(TorqueCurrentFOC.class));
        assertTrue(requestTypes.contains(Follower.class));

        Method brake = ExampleSubsystem.class.getMethod("setBrakeMode");
        Method coast = ExampleSubsystem.class.getMethod("setCoastMode");
        assertEquals(void.class, brake.getReturnType());
        assertEquals(void.class, coast.getReturnType());
    }
}
