package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Volts;

import java.security.AlgorithmConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {
    private final ArmFeedforward m_armFeedForward;
    private final TalonFX m_armMotor;
    private final TalonFXSimState m_armMotorSim;
    private final TalonFXConfiguration m_motorConfig;
    private final MotionMagicVoltage m_request;
    private final VoltageOut m_voltReq;
    private final SingleJointedArmSim m_armSim;
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_mech2dRoot;
    private final MechanismLigament2d m_mechLig2d;
    private final SysIdRoutine m_sysID;
    private final StatusSignal<Angle> m_positionSignal;
    private ArmPosition m_armPosition = ArmPosition.HOMED;

    public enum ArmPosition {
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        ALGAE_L2,
        ALGAE_L3,
        PROCESSOR,
        BARGE,
        HOMED
    }

    public ArmSubsystem() {
        m_armMotor = new TalonFX(ArmConstants.kArmMotorID);
        m_armMotorSim = m_armMotor.getSimState();
        m_motorConfig = new TalonFXConfiguration();
        m_armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

        /** Voltage control request object for the TalonFX motor controller */
        m_voltReq = new VoltageOut(0.0);
        m_request = new MotionMagicVoltage(0).withSlot(0);
        m_positionSignal = m_armMotor.getPosition();

        m_armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.kGearRatio, 
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(ArmConstants.kArmLength), ArmConstants.kArmMass),
            ArmConstants.kArmLigLength, 
            ArmConstants.kArmMinRads, 
            ArmConstants.kArmMaxRads,
            true,
            ArmConstants.kArmStartingRad);
        
        m_mech2d = new Mechanism2d(1.0, 1.0);
        m_mech2dRoot = m_mech2d.getRoot("Arm Root", 0.5, 0.5);
        m_mechLig2d = m_mech2dRoot.append(
            new MechanismLigament2d("Arm", ArmConstants.kArmLigLength, 90));
            
        m_mechLig2d.setLength(ArmConstants.kArmLigLength);

        /** Object of a system identification routine */
        m_sysID = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null, this));

        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kGearRatio;
        m_motorConfig.Feedback.RotorToSensorRatio = 1;

        m_motorConfig.Slot0.kP = ArmConstants.kP;
        m_motorConfig.Slot0.kI = ArmConstants.kI;
        m_motorConfig.Slot0.kD = ArmConstants.kD;
        m_motorConfig.Slot0.kS = ArmConstants.kS;
        m_motorConfig.Slot0.kV = ArmConstants.kV;

        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kArmMaxVelocity;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.kArmMaxAcceleration;

                // Set safety limits
        // m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        //         ArmConstants.kArmMaxAngleRad
        //                 / ArmConstants.kArmMetersPerMotorRotation;
        // m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        // Set current limits
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 50;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        m_armMotor.getConfigurator().apply(m_motorConfig);

        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Arm Sim", m_mech2d); 
        }
    }

    public void setArmEnumPosition(ArmPosition newArmPosition) {
        m_armPosition = newArmPosition;
    }

    public ArmPosition getArmPositionEnum() {
        return m_armPosition;
    }

    public double getArmPosition() {
        return m_positionSignal.getValueAsDouble();
    }

    public double getArmRotationDegrees() {
        return getArmPosition() * 360;
    }

    public boolean elevatorCanMove() {
        return getArmRotationDegrees() < 55.0;
    }

    public void goToPosition(double position) {
        double rotations = Math.toRadians(position) / (2.0 * Math.PI);
        m_request.Slot = 0;
        m_request.Position = rotations;
        m_armMotor.setControl(m_request);
    }

    public void setPosition(ArmPosition level) {
        double angle = 0;
        switch (level) {
            case CORAL_L1:
                angle = ArmConstants.kCoralL1;
                break;
            case CORAL_L2:
                angle = ArmConstants.kCoralL2;
                break;
            case CORAL_L3:
                angle = ArmConstants.kCoralL3;
                break;
            case CORAL_L4:
                angle = ArmConstants.kCoralL4;
                break;
            case ALGAE_L2:
                angle = ArmConstants.kHomed;
                break;
            case ALGAE_L3:
                angle = ArmConstants.kHomed;
                break;
            case BARGE:
                angle = ArmConstants.kBarge;
                break;
            case PROCESSOR:
                angle = ArmConstants.kProcessor;
                break;
            case HOMED:
                angle = ArmConstants.kHomed;
                break;                
        }
        setArmEnumPosition(level);
        goToPosition(angle);
    }

    public Command setAngleCommand(ArmPosition level) {
        return runOnce(() -> setPosition(level));
    }

    public void setVoltage(double voltage) {
        m_armMotor.setVoltage(voltage);
    }

        /**
     * Returns a command that will execute a quasistatic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysID.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysID.dynamic(direction);
    }

    @Override
    public void simulationPeriodic() {
        m_armMotorSim.setSupplyVoltage(12);
        m_armSim.setInput(m_armMotorSim.getMotorVoltage());
        m_armSim.update(0.02);

        m_armMotorSim.setRawRotorPosition(
            (m_armSim.getAngleRads() - ArmConstants.kArmMinRads)
                * ArmConstants.kGearRatio
                / (2.0
                * Math.PI));
    
        m_armMotorSim.setRotorVelocity(
            m_armSim.getVelocityRadPerSec() * ArmConstants.kGearRatio / (2.0 * Math.PI));
    
        m_mechLig2d.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_positionSignal);
        SmartDashboard.putString("Arm Enum Position", getArmPositionEnum().toString());
        SmartDashboard.putNumber("Arm Rotation", getArmRotationDegrees());
    }
    
}
