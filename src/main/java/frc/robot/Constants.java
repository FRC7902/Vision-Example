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
    public static double kPX = 3.2;
    public static double kIX = 0;
    public static double kDX = 0.01;

    public static double kPY = 5;
    public static double kIY = 0;
    public static double kDY = 0.2;

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
    public static final double kElevatorGearing = 4.875;
    public static final double kElevatorCarriageMass = Units.lbsToKilograms(24);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2);
    public static final double kElevatorMetersPerMotorRotation = (kElevatorDrumRadius * 2 * Math.PI)
            / kElevatorGearing;

    // Elevator Dimensions
    public static final double kElevatorHeightMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(73.5);

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
    public static final double kElevatorCoralStationAndProcessorHeight = Units.inchesToMeters(22.5);
    public static final double kElevatorCoralLevel1Height = Units.inchesToMeters(26.75);
    public static final double kElevatorCoralLevel2Height = Units.inchesToMeters(27.5);
    public static final double kElevatorCoralLevel3Height = Units.inchesToMeters(44);
    public static final double kElevatorCoralLevel4Height = Units.inchesToMeters(73.5);

    public static final double kElevatorAlgaeLowHeight = Units.inchesToMeters(28.5);
    public static final double kElevatorAlgaeHighHeight = Units.inchesToMeters(44.25);
    public static final double kElevatorAlgaeBargeHeight = Units.inchesToMeters(73.5);
    // ==============================

    // ===== Control Parameters =====
    public static final double kElevatorTargetError = 0.005;
    public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
                                                                 // voltage drop calculation
  }

  public static class ArmConstants {
    // PID
    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;

    // Feed-forward
    public static double kS = 0;
    public static double kG = 0;
    public static double kV = 0;
    public static double kA = 0;

    // Motor ID
    public static final int kArmMotorID = 30;

    // Physical Properties
    public static final double kGearRatio = 67.5;
    public static final double kArmMOI = 5;
    public static final double kArmLigLength =  0.5;
    public static final double kArmLength = 7.805; //inches
    public static final double kArmWidth = 1.25; //
    public static final double kArmMass = 3.40194;
    
    // Simulation Constants
    public static final double kArmMinRads = Math.toRadians(266);
    public static final double kArmMaxRads = Math.toRadians(450);
    public static final double kArmStartingRad = 0;

    // Limits
    public static final double kArmMaxVelocity = 10;
    public static final double kArmMaxAcceleration = 20;
    public static final double kArmMaxAngleRad = Math.toRadians(180);

    // Conversions
    public static final double kArmMetersPerMotorRotation = (2 * Math.PI) / kGearRatio;

    // Setpoints
    public static final double kHomed = 0;
    public static final double kCoralL1 = 135;
    public static final double kCoralL2 = 170;
    public static final double kCoralL3 = 170;
    public static final double kCoralL4 = 150;
    public static final double kBarge = 145;
    public static final double kProcessor = 45;
  }

}
