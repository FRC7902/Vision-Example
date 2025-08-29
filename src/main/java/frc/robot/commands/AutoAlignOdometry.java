// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignOdometry extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final SwerveDrive m_swerveDrive;

  private final LimelightSubsystem m_limelightSubsystem;

  // private final Trajectory m_trajectory;

  private final double reefOffset;
  private Rotation2d aprilTagRotation;

  private final HolonomicDriveController m_drivePID;

  /** Creates a new ArcadeDriveCommand. */
  public AutoAlignOdometry(SwerveSubsystem m_swerveSubsystem, LimelightSubsystem m_limelightSubsystem, ReefSide reefSide) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_limelightSubsystem = m_limelightSubsystem;
    m_swerveDrive = m_swerveSubsystem.getSwerveDrive();

    PIDController m_xController = new PIDController(VisionConstants.kPXFar, VisionConstants.kIXFar, VisionConstants.kDXFar);
    PIDController m_yController = new PIDController(VisionConstants.kPYFar, VisionConstants.kIYFar, VisionConstants.kDYFar);
    ProfiledPIDController m_thetaController = new ProfiledPIDController(VisionConstants.kPTheta, VisionConstants.kITheta, VisionConstants.kDTheta, null);

    m_drivePID = new HolonomicDriveController(m_xController, m_yController, m_thetaController);

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

    SmartDashboard.putNumber("KPX Close", VisionConstants.kPXClose);
    SmartDashboard.putNumber("KIX Close", VisionConstants.kIXClose);
    SmartDashboard.putNumber("KDX Close", VisionConstants.kDXClose);

    SmartDashboard.putNumber("KPY Close", VisionConstants.kPYClose);
    SmartDashboard.putNumber("KIY Close", VisionConstants.kIYClose);
    SmartDashboard.putNumber("KDY Close", VisionConstants.kDYClose);

    SmartDashboard.putNumber("KPX Far", VisionConstants.kPXFar);
    SmartDashboard.putNumber("KIX Far", VisionConstants.kIXFar);
    SmartDashboard.putNumber("KDX Far", VisionConstants.kDXFar);

    SmartDashboard.putNumber("KPY Far", VisionConstants.kPYFar);
    SmartDashboard.putNumber("KIY Far", VisionConstants.kIYFar);
    SmartDashboard.putNumber("KDY Far", VisionConstants.kDYFar);

    SmartDashboard.putNumber("PID X DIFF", VisionConstants.kPIDDifferenceConstantX);
    SmartDashboard.putNumber("PID Y DIFF", VisionConstants.kPIDDifferenceConstantY);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    List<Pose2d> reefPoses = m_limelightSubsystem.getReefPoses();
    Pose2d robotPose = m_swerveDrive.getPose();
    Pose2d closestAprilTag = robotPose.nearest(reefPoses);

    aprilTagRotation = closestAprilTag.getRotation();

    // aprilTagRotation = closestAprilTag.getRotation().getRadians();

    // double multiplier = Math.round(aprilTagRotation / Math.abs(aprilTagRotation));

    // if (aprilTagRotation == 0) {
    //   aprilTagRotation = Math.PI;
    // }
    
    // else {
    //   aprilTagRotation -= (Math.PI * multiplier);
    // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = m_swerveDrive.getPose();
    // Trajectory.State goal = m_trajectory.sample(3.4);

    ChassisSpeeds targetSpeed = m_drivePID.calculate(robotPose, null, aprilTagRotation);

    m_swerveDrive.drive(targetSpeed);

    // SmartDashboard.putNumber("X error", m_xController.getPositionError());
    // SmartDashboard.putNumber("Y error", m_yController.getPositionError());
    // SmartDashboard.putNumber("Robot Rotation", robotRotation);
    // SmartDashboard.putNumber("April Tag Rotation", aprilTagRotation);

    // SmartDashboard.putNumber("Y Speed", ySpeed);
    // SmartDashboard.putNumber("Omega Speed", ySpeed);

    updatePID();

  }

  void updatePID() {


    VisionConstants.kPXClose = SmartDashboard.getNumber("KPX Close", VisionConstants.kPXClose);
    VisionConstants.kIXClose = SmartDashboard.getNumber("KIX Close", VisionConstants.kIXClose);
    VisionConstants.kDXClose = SmartDashboard.getNumber("KDX Close", VisionConstants.kDXClose);

    VisionConstants.kPYClose = SmartDashboard.getNumber("KPY Close", VisionConstants.kPYClose);
    VisionConstants.kIYClose = SmartDashboard.getNumber("KIY Close", VisionConstants.kIYClose);
    VisionConstants.kDYClose = SmartDashboard.getNumber("KDY Close", VisionConstants.kDYClose);

    VisionConstants.kPXFar = SmartDashboard.getNumber("KPX Far", VisionConstants.kPXFar);
    VisionConstants.kIXFar = SmartDashboard.getNumber("KIX Far", VisionConstants.kIXFar);
    VisionConstants.kDXFar = SmartDashboard.getNumber("KDX Far", VisionConstants.kDXFar);

    VisionConstants.kPYFar = SmartDashboard.getNumber("KPY Far", VisionConstants.kPYFar);
    VisionConstants.kIYFar = SmartDashboard.getNumber("KIY Far", VisionConstants.kIYFar);
    VisionConstants.kDYFar = SmartDashboard.getNumber("KDY Far", VisionConstants.kDYFar);

    VisionConstants.kPIDDifferenceConstantX = SmartDashboard.getNumber("PID X DIFF", VisionConstants.kPIDDifferenceConstantX);
    VisionConstants.kPIDDifferenceConstantY = SmartDashboard.getNumber("PID Y DIFF", VisionConstants.kPIDDifferenceConstantY);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivePID.atReference(); 
}
}
