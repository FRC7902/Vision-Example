package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Volts;

import java.security.AlgorithmConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {
    private final ArmFeedforward m_armFeedForward;
    private final TalonFX m_armMotor;
    private TalonFXSimState m_armMotorSim;
    private final TalonFXConfiguration m_motorConfig;
    private final MotionMagicVoltage m_request;
    private final VoltageOut m_voltReq;
    private  SingleJointedArmSim m_armSim;
    private  Mechanism2d m_mech2d;
    private  MechanismRoot2d m_mech2dRoot;
    private  MechanismLigament2d m_mechLig2d;
    private final SysIdRoutine m_sysID;
    private final StatusSignal<Angle> m_positionSignal;
    private final CANcoder m_armEncoder;
    private  CANcoderSimState m_armEncoderSim;
    private ArmPosition m_armPosition = ArmPosition.HOMED;
    private ArmPosition m_desiredPosition = ArmPosition.HOMED;

    public enum ArmPosition {
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        ALGAE_L2,
        ALGAE_L3,
        PROCESSOR,
        BARGE,
        HOMED,
        READY
    }

    /**
     * Creates the {@code ArmSubsystem} object. This is where the arm and the arm encoder data is processed. 
    */ 
    public ArmSubsystem() {
        /** Initialization of motor and encoder objects */
        m_armMotor = new TalonFX(ArmConstants.kArmMotorID);
        m_motorConfig = new TalonFXConfiguration();
        m_armFeedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
        m_armEncoder = new CANcoder(ArmConstants.kArmEncoderID);

        /** Voltage control request object for the TalonFX motor controller to be used for SysId
         *  Motion Magic control request object for the TalonFX motor controller for position control
        */
        m_voltReq = new VoltageOut(0.0);
        m_request = new MotionMagicVoltage(0).withSlot(0);
        m_positionSignal = m_armMotor.getPosition();

        /** Object of a system identification routine */
        m_sysID = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> m_armMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null, this));

        /** Set encoder to the external CANCoder */
        m_motorConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.kArmEncoderID;
        m_motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        /** Set motor configuration */
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kGearRatio;
        m_motorConfig.Feedback.RotorToSensorRatio = 1;

        /** Set PID and FeedForward Constants */
        m_motorConfig.Slot0.kP = ArmConstants.kP;
        m_motorConfig.Slot0.kI = ArmConstants.kI;
        m_motorConfig.Slot0.kD = ArmConstants.kD;
        m_motorConfig.Slot0.kS = ArmConstants.kS;
        m_motorConfig.Slot0.kV = ArmConstants.kV;
        m_motorConfig.Slot0.kA = ArmConstants.kA;

        /** Set gravity type */
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        /** Set maximum velocity and acceleration to be used for Motion Magic position control */
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kArmMaxVelocity;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.kArmMaxAcceleration;

                // Set safety limits
        // m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        //         ArmConstants.kArmMaxAngleRad
        //                 / ArmConstants.kArmMetersPerMotorRotation;
        // m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        /** Set current limits */
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 50;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        /** Applies configuration to motor */
        m_armMotor.getConfigurator().apply(m_motorConfig);

        /** Initialization of the Arm Sim, Motor Sim, and Encoder Sim when the Robot is detected to be in simulation */
        if (Robot.isSimulation()) {
            /** Creates a new Arm Sim.*/
            m_armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ArmConstants.kGearRatio, 
                SingleJointedArmSim.estimateMOI(Units.inchesToMeters(ArmConstants.kArmLength), ArmConstants.kArmMass),
                ArmConstants.kArmLigLength, 
                ArmConstants.kArmMinRads, 
                ArmConstants.kArmMaxRads,
                true,
                ArmConstants.kArmStartingRad);
            
            /** Creates the visual aspect of the Arm Sim.*/
            m_mech2d = new Mechanism2d(1.0, 1.0);
            m_mech2dRoot = m_mech2d.getRoot("Arm Root", 0.5, 0.5);
            m_mechLig2d = m_mech2dRoot.append(
                new MechanismLigament2d("Arm", ArmConstants.kArmLigLength, 90));
            m_mechLig2d.setLength(ArmConstants.kArmLigLength);

            /** Gets the simulation objects from the arm motor and encoder.*/
            m_armMotorSim = m_armMotor.getSimState();
            m_armEncoderSim = m_armEncoder.getSimState();

            /** Displays the visual arm sim (visible in Glass during simulation).*/
            SmartDashboard.putData("Arm Sim", m_mech2d); 
        }
    }

    /**
     * Sets the enumeration position of the arm.
     * @param newArmPosition The position the arm is at using the {@code ArmPosition} enumeration.
    */     
    public void setArmEnumPosition(ArmPosition newArmPosition) {
        m_armPosition = newArmPosition;
    }

    /**
     * Gets the position of the arm in the form of the {@code ArmPosition} enumeration.
     * @return The enumeration position of the arm.
    */    
    public ArmPosition getArmPositionEnum() {
        return m_armPosition;
    }

    /**
     * Sets the desired position of the arm in the form of the {@code ArmPosition} enumeration.
     * This is used for scheduling arm positions, to be used in auto-scoring.
     * @param desiredPosition The position the arm should go to during auto-scoring.
    */        
    public void setArmDesiredPosition(ArmPosition desiredPosition) {
        m_desiredPosition = desiredPosition;
    }

    /**
     * Gets the position of the arm as a double.
     * @return The position of the arm.
    */       
    public double getArmPosition() {
        return m_positionSignal.getValueAsDouble();
    }

    /**
     * Gets the rotation of the arm in degrees.
     * @return The rotation of the arm.
    */        
    public double getArmRotationDegrees() {
        return getArmPosition() * 360;
    }

    /**
     * Determines whether the arm is at an angle where the elevator can safely move.
     * @return Whether the elevator can move or not.
    */        
    public boolean elevatorCanMove() {
        return getArmRotationDegrees() < 55.0 ;
    }

    /**
     * Commands the motor to move the arm to the given angle. The angle is converted into rotations (required for CTRE devices).
     * @param angle The angle the arm should be at.
    */       
    public void goToAngle(double angle) {
        double rotations = Math.toRadians(angle) / (2.0 * Math.PI);
        m_request.Slot = 0;
        m_request.Position = rotations;
        m_armMotor.setControl(m_request);
    }

    /**
     * Determines the angle value of the arm based on the {@code ArmPosition} enumeration given.
     * @param angle The position of the arm using the {@code ArmPosition} enumeration.
    */         
    public void setAngle(ArmPosition angle) {
        double requestedAngle = switch (angle) {
            case CORAL_L1 -> ArmConstants.kCoralL1;
            case CORAL_L2 -> ArmConstants.kCoralL2;
            case CORAL_L3 -> ArmConstants.kCoralL3;
            case CORAL_L4 -> ArmConstants.kCoralL4;
            case BARGE -> ArmConstants.kBarge;
            case PROCESSOR -> ArmConstants.kProcessor;
            case READY -> ArmConstants.kReadyPos;
            default -> ArmConstants.kHomed;
        };
        setArmEnumPosition(angle);
        goToAngle(requestedAngle);
    }    

    /**
     * Command to ready the arm by moving it more outwards after receiving CORAL.
    */         
    public Command readyArm() {
        return runOnce(() -> setAngle(ArmPosition.READY)); 
    }

    /**
     * Command to rotate the arm to the set {@code ArmPosition}.
     * @param requestedAngle The position to rotate to.
    */   
    public Command rotateArm(ArmPosition requestedAngle) {
        return runOnce(() -> setAngle(requestedAngle));
    }

    /**
     * Rotates the arm using the scheduled arm height. Used in auto-score.
    */       
    public Command rotateArm() {
        // return runOnce(() -> setPosition(m_desiredPosition));
        return Commands.none().until(() -> RobotContainer.m_elevatorSubsystem.atSafeHeight()).finallyDo(() -> setAngle(m_desiredPosition));
    }

    /**
     * Manually applies voltage to motor. This should only be used for testing/debugging purposes.
     * @param voltage The voltage to provide to the motor.
    */   
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

    /** The simulation periodic loop, ran every 20 ms when the robot code is being simulated.*/
    @Override
    public void simulationPeriodic() {
        m_armMotorSim.setSupplyVoltage(12);
        m_armSim.setInput(m_armMotorSim.getMotorVoltage());
        m_armSim.update(0.02);

        double rawRotorPos = (m_armSim.getAngleRads() - ArmConstants.kArmMinRads) * ArmConstants.kGearRatio / (2 * Math.PI);
        double rotorVel = m_armSim.getVelocityRadPerSec() * ArmConstants.kGearRatio / (2.0 * Math.PI);

        m_armMotorSim.setRawRotorPosition(rawRotorPos);    
        m_armMotorSim.setRotorVelocity(rotorVel);
        m_armEncoderSim.setRawPosition(rawRotorPos);

        m_mechLig2d.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    /** The periodic loop, ran every 20 ms when the robot code is either real or being simulated.*/
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_positionSignal);
        SmartDashboard.putString("Arm Enum Position", getArmPositionEnum().toString());
        SmartDashboard.putNumber("Arm Rotation", getArmRotationDegrees());
    }    
}
