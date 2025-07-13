package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightSubsystem {
    
    private final DriveSubsystem m_driveSubsystem;
    private final String camera;

    public LimelightSubsystem(DriveSubsystem m_driveSubsystem, String camera) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.camera = camera;

        LimelightHelpers.SetIMUMode(camera, 1);
    }

    public boolean aprilTagDetected() {
        return LimelightHelpers.getTV(camera);
    }

}
