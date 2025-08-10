// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  }
  
  public static class PhotonConstants {
    public static final String middleCamName = "middleCam";
    public static final Transform3d middleCamToRobotTsf = 
      new Transform3d(0, 0, 0.35, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    public static final CameraProperties middleCamProp =
      new CameraProperties(middleCamName, middleCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    // TODO: Create new constants for a left camera
    // With an appropriate name,
    // With offsets: x = 0.3, y = 0.6, z = 0.35,
    // No rotation (0 for all angles)
    // with a resolution width of 640, a resolution height of 480,
    // an FOV of 100 degrees,
    // Average Error PX of 0.25,
    // Error Standard Deviation of 0.08

    public static final String leftCamName = "leftCam";
    public static final Transform3d leftCamToRobotTsf = 
      new Transform3d(0.3, 0.6, 0.35, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    public static final CameraProperties leftCamProp =
      new CameraProperties(leftCamName, leftCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

  }

  public static class LimelightConstants {
    public static final String leftCamName = "left";
    public static double kStdDevs = 0.800000;
  }

  public static class VisionConstants {
    // Contains the stored position of each April Tag on the field. This varies between seasons.
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }
}
