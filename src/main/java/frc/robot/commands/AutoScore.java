// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final SwerveDrive m_swerveDrive;
  private final SwerveController m_swerveController;

  private final PhotonSubsystem m_camera;
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;

  private final double reefOffset;
  private double aprilTagRotation;

  /** Creates a new ArcadeDriveCommand. */
  public AutoScore(SwerveSubsystem m_swerveSubsystem, PhotonSubsystem m_camera, ReefSide reefSide) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_camera = m_camera;

    m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
    m_swerveController = m_swerveDrive.getSwerveController();

    m_xController = new ProfiledPIDController(DriveConstants.kPX, DriveConstants.kIX, DriveConstants.kDX, DriveConstants.kXConstraints);
    m_yController = new ProfiledPIDController(DriveConstants.kPY, DriveConstants.kIY, DriveConstants.kDY, DriveConstants.kYConstraints);

    switch (reefSide) {
      case LEFT:
        reefOffset = -VisionConstants.aprilTagOffset;
        break;
      case RIGHT:
        reefOffset = VisionConstants.aprilTagOffset;;
        break;
      default:
        reefOffset = 0;  
    }

    SmartDashboard.putNumber("KPX", DriveConstants.kPX);
    SmartDashboard.putNumber("KIX", DriveConstants.kIX);
    SmartDashboard.putNumber("KDX", DriveConstants.kDX);

    SmartDashboard.putNumber("KPY", DriveConstants.kPY);
    SmartDashboard.putNumber("KIY", DriveConstants.kIY);
    SmartDashboard.putNumber("KDY", DriveConstants.kDY);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController.setGoal(VisionConstants.xOffset);
    m_yController.setGoal(reefOffset);

    m_xController.setTolerance(0.01);
    m_yController.setTolerance(0.01);

    aprilTagRotation = VisionConstants.aprilTagFieldLayout.getTagPose(m_camera.getTagID()).get().getRotation().getZ();

    double multiplier = Math.round(aprilTagRotation / Math.abs(aprilTagRotation));

    if (aprilTagRotation == 0) {
      aprilTagRotation = Math.PI;
    }
    
    else {
      aprilTagRotation = aprilTagRotation - (Math.PI * multiplier);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double aprilTagTX = m_camera.getTagTX();
    double aprilTagTY = m_camera.getTagTY();
    double aprilTagRot = m_camera.getTagRot();

    double robotRotation = m_swerveDrive.getPose().getRotation().getRadians();

    double xSpeed = m_xController.calculate(aprilTagTX);
    double ySpeed = m_yController.calculate(aprilTagTY);
    double omegaSpeed = m_swerveController.headingCalculate(robotRotation, aprilTagRotation);

    m_swerveDrive.drive(new ChassisSpeeds(-xSpeed, -ySpeed, omegaSpeed));

    SmartDashboard.putNumber("X error", m_xController.getPositionError());
    SmartDashboard.putNumber("Y error", m_yController.getPositionError());
    SmartDashboard.putNumber("Robot Rotation", robotRotation);
    SmartDashboard.putNumber("April Tag Rotation", aprilTagRotation);

    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Omega Speed", ySpeed);

    updatePID();

  }

  void updatePID() {

    DriveConstants.kPX = SmartDashboard.getNumber("KPX", DriveConstants.kPX);
    DriveConstants.kIX = SmartDashboard.getNumber("KIX", DriveConstants.kIX);
    DriveConstants.kDX = SmartDashboard.getNumber("KDX", DriveConstants.kDX);

    DriveConstants.kPY = SmartDashboard.getNumber("KPY", DriveConstants.kPY);
    DriveConstants.kIY = SmartDashboard.getNumber("KIY", DriveConstants.kIY);
    DriveConstants.kDY = SmartDashboard.getNumber("KDY", DriveConstants.kDY);

    m_xController.setPID(DriveConstants.kPX, DriveConstants.kIX, DriveConstants.kDX);
    m_yController.setPID(DriveConstants.kPY, DriveConstants.kIY, DriveConstants.kDY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atGoal() && m_yController.atGoal();
  }
}
