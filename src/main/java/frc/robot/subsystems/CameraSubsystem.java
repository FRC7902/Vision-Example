package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private final DriveSubsystem m_driveSubsystem;
    private final VisionSystemSim m_visionSim;
    private final TargetModel targetModel;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final PhotonCamera m_camera;
    private final PhotonCameraSim m_cameraSim;

    public CameraSubsystem(DriveSubsystem m_driveSubsystem) {

        this.m_driveSubsystem = m_driveSubsystem;

        m_visionSim = new VisionSystemSim("main");
        targetModel = TargetModel.kAprilTag36h11;
        m_visionSim.addAprilTags(aprilTagFieldLayout);

        SimCameraProperties cameraSettings = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraSettings.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraSettings.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraSettings.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraSettings.setAvgLatencyMs(35);
        cameraSettings.setLatencyStdDevMs(5);

        m_camera = new PhotonCamera("camera");
        m_cameraSim = new PhotonCameraSim(m_camera, cameraSettings);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        m_visionSim.addCamera(m_cameraSim, robotToCamera);

        m_cameraSim.enableDrawWireframe(true);

    }

    @Override
    public void simulationPeriodic() {

        Pose2d robotPose = m_driveSubsystem.getPose();

        var debugField = m_visionSim.getDebugField();
        debugField.getObject("EstimatedRobot").setPose(robotPose);

        m_visionSim.update(robotPose);

        // Calculate battery voltage sag due to current draw
        var batteryVoltage =
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveSubsystem.getCurrentAmpsDraw());

        // Using max(0.1, voltage) here isn't a *physically correct* solution,
        // but it avoids problems with battery voltage measuring 0.
        RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
    }

}
