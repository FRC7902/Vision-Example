// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final PhotonSubsystem m_camera;
  private final SwerveDrive m_swerveDrive;
  private final CommandXboxController m_driverController;
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final ProfiledPIDController m_omegaController;
  private double aprilTagRotation;

  private SwerveInputStream driveToPosition;

  /** Creates a new ArcadeDriveCommand. */
  public AutoScore(SwerveSubsystem m_swerveSubsystem, PhotonSubsystem m_camera, CommandXboxController m_driverController) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_camera = m_camera;
    this.m_driverController = m_driverController;

    m_xController = new ProfiledPIDController(DriveConstants.kPX, DriveConstants.kIX, DriveConstants.kDX, DriveConstants.kXConstraints);
    m_yController = new ProfiledPIDController(DriveConstants.kPY, DriveConstants.kIY, DriveConstants.kDY, DriveConstants.kYConstraints);
    m_omegaController = new ProfiledPIDController(DriveConstants.kPOmega, DriveConstants.kIOmega, DriveConstants.kDOmega, DriveConstants.kOmegaConstraints);


    m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
    addRequirements(m_swerveSubsystem);
    // TODO: Insert your constructor code here...
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController.setGoal(VisionConstants.xOffset);
    m_yController.setGoal(VisionConstants.yOffset);

    aprilTagRotation = VisionConstants.aprilTagFieldLayout.getTagPose(m_camera.getTagID()).get().getRotation().getZ();

    double multiplier = Math.round(aprilTagRotation / Math.abs(aprilTagRotation));

    if (aprilTagRotation == 0) {
      aprilTagRotation = Math.PI;
    }
    
    else {
      aprilTagRotation = aprilTagRotation - (Math.PI * multiplier);
    }

    m_omegaController.setGoal(aprilTagRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double aprilTagTX = m_camera.getTagTX();
    double aprilTagTY = m_camera.getTagTY();
    double aprilTagRot = m_camera.getTagRot();

    Pose2d robotPose = m_swerveDrive.getPose();

    double xSpeed = m_xController.calculate(aprilTagTX);
    double ySpeed = m_yController.calculate(aprilTagTY);
    double omegaSpeed = m_omegaController.calculate(robotPose.getRotation().getRadians());

    // m_swerveDrive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, omegaSpeed));
    m_swerveDrive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, 0));

    SmartDashboard.putNumber("X error", m_xController.getPositionError());
    SmartDashboard.putNumber("Y error", m_yController.getPositionError());
    SmartDashboard.putNumber("Omega error", m_omegaController.getPositionError());
    SmartDashboard.putNumber("Robot Rotation", robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("April Tag Rotation", aprilTagRotation);

    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Omega Speed", ySpeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
