package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final TalonFX m_intakeMotor;
    private final TalonFXSimState m_intakeMotorSim;
    private final TalonFXConfiguration m_motorConfig;
    private final VoltageOut m_voltReq;
    private IntakeStatus m_intakeStatus = IntakeStatus.EMPTY;

    public enum IntakeStatus {
        INTAKING,
        OUTTAKING,
        CORAL_LOADED,
        ALGAE_LOADED,
        EMPTY
    }

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);
        m_intakeMotorSim = m_intakeMotor.getSimState();
        m_motorConfig = new TalonFXConfiguration();
        m_voltReq = new VoltageOut(0.0);

        // Set current limits
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 50;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_intakeMotor.getConfigurator().apply(m_motorConfig);
    }

    public IntakeStatus getIntakeStatus() {
        return m_intakeStatus;
    }

    public void setIntakeStatus(IntakeStatus newStatus) {
        m_intakeStatus = newStatus;
    }

    public void outtake() {
        m_intakeMotor.setControl(m_voltReq.withOutput(12));
    }

    public void intake() {
        m_intakeMotor.setControl(m_voltReq.withOutput(-12));
    }

    public void stopMotor() {
        m_intakeMotor.setControl(m_voltReq.withOutput(0));
    }

    public Command outtakeCommand() {
        return run(() -> outtake()).withTimeout(IntakeConstants.kIntakeOuttakeTimeoutSec).andThen(() -> stopMotor());        
    }

    public Command intakeCommand() {
        return run(() -> intake()).withTimeout(IntakeConstants.kIntakeOuttakeTimeoutSec).andThen(() -> stopMotor());
    }

    public Command stopCommand() {
        return runOnce(() -> stopMotor());
    }


}
