// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.vision.PhotonSim;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // TODO: Initialize your DriveSubsystem here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController m_driverController = new CommandXboxController(
        OperatorConstants.kDriverControllerPort);
        
  // public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
                          m_driverController,
                        new File(Filesystem.getDeployDirectory(), "swerve"));
                        
  public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  public final PhotonSubsystem m_leftCamera = new PhotonSubsystem(PhotonConstants.leftCamProp);    
  public final PhotonSubsystem m_rightCamera = new PhotonSubsystem(PhotonConstants.rightCamProp);    
  public final PhotonSubsystem m_middleCamera = new PhotonSubsystem(PhotonConstants.middleCamProp);

  public PhotonSim m_cameraSim;

  public SwerveInputStream driveAngularVelocity = SwerveInputStream
                .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
                .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
                .allianceRelativeControl(true);


  public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
  .allianceRelativeControl(false);                

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Robot.isSimulation()) {
      m_cameraSim = new PhotonSim(m_swerveSubsystem, m_leftCamera, m_middleCamera, m_rightCamera);  
    }
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, m_driverController));
    m_swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);
    m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.intakeCommand(RobotBase.isSimulation()));

    m_elevatorSubsystem.setElevatorDesiredPosition(ElevatorPosition.CORAL_L1);
    m_armSubsystem.setArmDesiredPosition(ArmPosition.CORAL_L1);

    m_driverController.a().onTrue(new InstantCommand(
      () -> m_elevatorSubsystem.setElevatorDesiredPosition(ElevatorPosition.CORAL_L1)).alongWith(
        new InstantCommand(() -> m_armSubsystem.setArmDesiredPosition(ArmPosition.CORAL_L1))));

    m_driverController.b().onTrue(new InstantCommand(
      () -> m_elevatorSubsystem.setElevatorDesiredPosition(ElevatorPosition.CORAL_L2)).alongWith(
        new InstantCommand(() -> m_armSubsystem.setArmDesiredPosition(ArmPosition.CORAL_L2))));

    m_driverController.x().onTrue(new InstantCommand(
      () -> m_elevatorSubsystem.setElevatorDesiredPosition(ElevatorPosition.CORAL_L3)).alongWith(
        new InstantCommand(() -> m_armSubsystem.setArmDesiredPosition(ArmPosition.CORAL_L3))));

    m_driverController.y().onTrue(new InstantCommand(
      () -> m_elevatorSubsystem.setElevatorDesiredPosition(ElevatorPosition.CORAL_L4)).alongWith(
        new InstantCommand(() -> m_armSubsystem.setArmDesiredPosition(ArmPosition.CORAL_L4))));      

    m_driverController.rightTrigger(0.05).whileTrue(
      new SequentialCommandGroup(
        new AutoAlign(m_swerveSubsystem, m_middleCamera, ReefSide.RIGHT),
        new ParallelCommandGroup(
          m_elevatorSubsystem.goToPosition(),
          m_armSubsystem.rotateArm()),
        m_intakeSubsystem.outtakeCommand(RobotBase.isSimulation()),
        new ParallelCommandGroup(
          m_intakeSubsystem.stopCommand(),
          m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_STATION_AND_PROCESSOR),
          m_armSubsystem.rotateArm(ArmPosition.HOMED)))
    ).onFalse(

      new ParallelCommandGroup(
        m_intakeSubsystem.stopCommand(),
        m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_STATION_AND_PROCESSOR),
        m_armSubsystem.rotateArm(ArmPosition.HOMED)));

    m_driverController.leftTrigger(0.05).whileTrue(
      new SequentialCommandGroup(
        new AutoAlign(m_swerveSubsystem, m_middleCamera, ReefSide.LEFT),
        new ParallelCommandGroup(
          m_elevatorSubsystem.goToPosition(),
          m_armSubsystem.rotateArm()),
        m_intakeSubsystem.outtakeCommand(RobotBase.isSimulation()),
        new ParallelCommandGroup(
          m_intakeSubsystem.stopCommand(),
          m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_STATION_AND_PROCESSOR),
          m_armSubsystem.rotateArm(ArmPosition.HOMED)))
    ).onFalse(
      new ParallelCommandGroup(
        m_intakeSubsystem.stopCommand(),
        m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_STATION_AND_PROCESSOR),
        m_armSubsystem.rotateArm(ArmPosition.HOMED)));        

    // m_driverController.leftTrigger(0.05).whileTrue(new SequentialCommandGroup(
    //   new AutoScore(m_swerveSubsystem, m_middleCamera, ReefSide.LEFT),
    //   new ParallelCommandGroup(
    //     m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_L4),
    //     m_armSubsystem.setAngleCommand(ArmPosition.CORAL_L4))));

    // m_driverController.a().whileTrue(m_elevatorSubsystem.goToPosition(ElevatorPosition.ALGAE_HIGH));
    // m_driverController.b().whileTrue(m_elevatorSubsystem.goToPosition(ElevatorPosition.CORAL_STATION_AND_PROCESSOR));


    // m_driverController.a().onTrue(m_armSubsystem.rotateArm(ArmPosition.CORAL_L1));
    // m_driverController.b().onTrue(m_armSubsystem.rotateArm(ArmPosition.CORAL_L2));
    // m_driverController.x().onTrue(m_armSubsystem.rotateArm(ArmPosition.CORAL_L3));
    // m_driverController.y().onTrue(m_armSubsystem.rotateArm(ArmPosition.CORAL_L4));

    m_driverController.povDown().onTrue(m_armSubsystem.rotateArm(ArmPosition.HOMED));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
