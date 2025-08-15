// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.15;
  }

  public static class DriveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(15);
    public static final double kPX = 3.5;
    public static final double kIX = 0;
    public static final double kDX = 0;

    public static final double kPY = 3.5;
    public static final double kIY = 0;
    public static final double kDY = 0;

    public static final Constraints kXConstraints = new Constraints(20,   20);
    public static final Constraints kYConstraints = new Constraints(20, 20);
  }
  
  public static class PhotonConstants {
    public static final String leftCamName = "left";
    public static final Transform3d leftCamToRobotTsf = 
      new Transform3d(0.207, 0.150, 0.567, new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(-4.333)));
    public static final CameraProperties leftCamProp = 
      new CameraProperties(leftCamName, leftCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String rightCamName = "right";
    public static final Transform3d rightCamToRobotTsf = 
      new Transform3d(0.207, -0.150, 0.567, new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(4.333)));
    public static final CameraProperties rightCamProp = 
      new CameraProperties(rightCamName, rightCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String middleCamName = "middle";
    public static final Transform3d middleCamToRobotTsf = 
      new Transform3d(0, 0, 0.35, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    public static final CameraProperties middleCamProp =
      new CameraProperties(middleCamName, middleCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);
  }

  public static class LimelightConstants {
    public static final String leftCamName = "left";
    public static double kStdDevs = 0.800000;
  }

  public static class VisionConstants {
    // Contains the stored position of each April Tag on the field. This varies between seasons.
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double xOffset = 0.8;
    public static final double aprilTagOffset = 0.1;
  }

  public static final class ElevatorConstants {
    // CAN IDs
    public static final int kElevatorLeaderCAN = 54;
    public static final int kElevatorFollowerCAN = 55;

    // Physical Constants
    public static final double kElevatorGearing = 7.5;
    public static final double kElevatorCarriageMass = Units.lbsToKilograms(20);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.644 / 2);
    public static final double kElevatorMetersPerMotorRotation = (kElevatorDrumRadius * 2 * Math.PI)
            / kElevatorGearing;

    // Elevator Dimensions
    public static final double kElevatorHeightMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.90;

    // Motion Constraints
    public static final double kElevatorMaxVelocity = 1.5 / ElevatorConstants.kElevatorMetersPerMotorRotation;
    public static final double kElevatorMaxAcceleration = 160.0;

    // PID Constants
    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.01;

    // Elevator Gains
    // set all to 0 during testing
    public static final double kElevatorS = 0.0; // negligible
    public static final double kElevatorG = 0.2;
    public static final double kElevatorV = 6.85 * kElevatorMetersPerMotorRotation;
    public static final double kElevatorA = 0.04 * kElevatorMetersPerMotorRotation;

    // ===== Elevator Setpoints =====
    public static final double kElevatorCoralStationAndProcessorHeight = 0.0;

    public static final double kElevatorCoralLevel1StartHeight = 0.025;
    public static final double kElevatorCoralLevel1EndHeight = 0.225;
    public static final double kElevatorCoralLevel2Height = 0.188;
    public static final double kElevatorCoralLevel3Height = 0.548;

    public static final double kElevatorAlgaeLowHeight = 0.604;
    public static final double kElevatorAlgaeHighHeight = 0.90;
    // ==============================

    // ===== Control Parameters =====
    public static final double kElevatorTargetError = 0.005;
    public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
                                                                 // voltage drop calculation
}
}
