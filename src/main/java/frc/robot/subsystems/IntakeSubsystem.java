    package frc.robot.subsystems;

    import com.ctre.phoenix6.configs.TalonFXConfiguration;
    import com.ctre.phoenix6.controls.VoltageOut;
    import com.ctre.phoenix6.hardware.TalonFX;
    import com.ctre.phoenix6.signals.InvertedValue;
    import com.ctre.phoenix6.signals.NeutralModeValue;
    import com.ctre.phoenix6.sim.TalonFXSimState;

    import edu.wpi.first.math.controller.ArmFeedforward;
    import edu.wpi.first.wpilibj.DigitalInput;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.wpilibj2.command.WaitCommand;
    import frc.robot.RobotContainer;
    import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
    import frc.robot.Constants.ArmConstants;
    import frc.robot.Constants.IntakeConstants;

    public class IntakeSubsystem extends SubsystemBase {
        
        private final TalonFX m_intakeMotor;
        private final TalonFXSimState m_intakeMotorSim;
        private final TalonFXConfiguration m_motorConfig;
        private final VoltageOut m_voltReq;
        private final DigitalInput m_deepBeamBreak;
        private final DigitalInput m_shallowBeamBreak;
        private final DigitalInput m_capacitiveSensor;
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
            m_deepBeamBreak = new DigitalInput(IntakeConstants.kDeepBeamBreakID);
            m_shallowBeamBreak = new DigitalInput(IntakeConstants.kShallowBeamBreakID);
            m_capacitiveSensor = new DigitalInput(IntakeConstants.kCapacitiveSensorID);

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

        public boolean isDeepBeamBreakBroken() {
            return m_deepBeamBreak.get() == true;
        }

        public boolean isShallowBeamBreakBroken() {
            return m_shallowBeamBreak.get() == true;
        }

        public boolean algaeIsPresent() {
            return m_capacitiveSensor.get() == true;
        }

        public void outtake() {
            m_intakeMotor.setControl(m_voltReq.withOutput(12));
            setIntakeStatus(IntakeStatus.OUTTAKING);
        }

        public void intakeCoral() {
            if (!isShallowBeamBreakBroken() && !isDeepBeamBreakBroken()) {
                m_intakeMotor.setControl(m_voltReq.withOutput(12));
            }

            else if (isShallowBeamBreakBroken() && !isDeepBeamBreakBroken()) {
                m_intakeMotor.setControl(m_voltReq.withOutput(6));
                setIntakeStatus(IntakeStatus.INTAKING);
            }

            else if (!isShallowBeamBreakBroken() && isDeepBeamBreakBroken()) {
                m_intakeMotor.setControl(m_voltReq.withOutput(-12));
                setIntakeStatus(IntakeStatus.INTAKING);
            }

            else if (isShallowBeamBreakBroken() && isDeepBeamBreakBroken()) {
                stopMotor();
                setIntakeStatus(IntakeStatus.CORAL_LOADED);
            }

        }

        public void intakeAlgae() {
            if (!algaeIsPresent()) {
                m_intakeMotor.setControl(m_voltReq.withOutput(12));
            }

            else if (algaeIsPresent()) {
                stopMotor();
                setIntakeStatus(IntakeStatus.ALGAE_LOADED);
            }     
        }

        public void stopMotor() {
            m_intakeMotor.setControl(m_voltReq.withOutput(0));
        }

        public Command outtakeCommand(boolean isSim) {
            
            ElevatorPosition m_elevatorPosition = RobotContainer.m_elevatorSubsystem.getElevatorEnumPosition();

            if (isSim) {
                return run(() -> outtake()).withTimeout(IntakeConstants.kIntakeOuttakeTimeoutSec).andThen(() -> stopMotor());
            }
            
            else if (m_elevatorPosition == ElevatorPosition.ALGAE_LOW || m_elevatorPosition == ElevatorPosition.ALGAE_HIGH) {
                return run(() -> outtake()).until(() -> (!algaeIsPresent())).finallyDo(() -> stopMotor());
            }

            else {
                return run(() -> outtake()).until(() -> (!isShallowBeamBreakBroken() && !isDeepBeamBreakBroken())).finallyDo(() -> stopMotor());
            }
        }

        public Command intakeCommand(boolean isSim) {

            ElevatorPosition m_elevatorPosition = RobotContainer.m_elevatorSubsystem.getElevatorEnumPosition();

            if (isSim && m_elevatorPosition == ElevatorPosition.ALGAE_LOW || m_elevatorPosition == ElevatorPosition.ALGAE_HIGH) {
                return run(() -> intakeAlgae()).withTimeout(IntakeConstants.kIntakeOuttakeTimeoutSec).andThen(() -> stopMotor());
            }

            else if (isSim) {
                return run(() -> intakeCoral()).withTimeout(IntakeConstants.kIntakeOuttakeTimeoutSec).andThen(() -> stopMotor());
            }
            
            else if (m_elevatorPosition == ElevatorPosition.ALGAE_LOW || m_elevatorPosition == ElevatorPosition.ALGAE_HIGH) {
                return run(() -> intakeAlgae()).until(() -> (algaeIsPresent())).finallyDo(() -> stopMotor());
            }

            else {
                return run(() -> intakeCoral()).until(() -> (isShallowBeamBreakBroken() && isDeepBeamBreakBroken())).finallyDo(() -> stopMotor());
            }
        }


        public Command stopCommand() {
            return runOnce(() -> stopMotor());
        }

        public void updateOdometry() {
            SmartDashboard.putNumber("Intake Motor Output", m_intakeMotor.getMotorVoltage().getValueAsDouble());
        }

        @Override
        public void periodic() {
            updateOdometry();
        }

        @Override
        public void simulationPeriodic() {
            m_intakeMotorSim.setSupplyVoltage(12);
        }

    }
