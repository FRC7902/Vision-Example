package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PhotonSubsystem extends SubsystemBase {

    private DriveSubsystem m_driveSubsystem;
    private VisionSystemSim m_visionSim;
    private TargetModel targetModel;
    private PhotonCamera m_camera;
    private PhotonCameraSim m_cameraSim;
    private DetectionStatus detectionStatus = DetectionStatus.NONE;

    private double aprilTagRot = 0;
    private double aprilTagTx = 0;
    private double aprilTagTy = 0;
    private double aprilTagID = 0;
    private double aprilTagArea = 0;
    private int detectedTagsCount = 0;

    // Contains the stored position of each April Tag on the field. This varies between seasons.
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public PhotonSubsystem(DriveSubsystem m_driveSubsystem) {
        
        // Creates the PhotonVision camera object. This is the non-sim object. In simulation, it is just used as a reference.
        // Be sure the camera name matches the name set in PhotonVision. Otherwise, it will not detect the camera.
        m_camera = new PhotonCamera(PhotonConstants.cameraName);

        // Determines if the robot code is being simulated. 
        // Helps reduce memory waste on simulated object if the robot code is real (running on the robot).
        if (Robot.isSimulation()) {

            // Instantiate DriveSubsystem object, used for obtaining robot pose.
            this.m_driveSubsystem = m_driveSubsystem;

            // Creates the Vision Simulation Object
            m_visionSim = new VisionSystemSim("main");

            // Used to set the dimensions of the target model. In this instance, there is already a pre-set for the current April Tag dimensions.
            targetModel = TargetModel.kAprilTag36h11;

            // Adds the position of the April Tags onto the vision simulator. Used for viewing the april tags from the simulated camera view.
            m_visionSim.addAprilTags(aprilTagFieldLayout);

            // Creates a simulation camera configurator object, which is used to set some settings for the camera.
            // Ideally should match the specifications of the cameras that are on the robot.
            SimCameraProperties cameraSettings = new SimCameraProperties();

            // Sets the resolution of the camera.
            // For example, a 640 x 480 camera with a 100 degree diagonal FOV.
            cameraSettings.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Sets the approximate detection noise with average and standard deviation error in pixels.
            // This can be used to determine how much calibration error can affect reliability (can use the calibration error from the actual camera calibration results).
            cameraSettings.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraSettings.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            cameraSettings.setAvgLatencyMs(35);
            cameraSettings.setLatencyStdDevMs(5);

            // Creates the simulated PhotonVision camera object.
            // Uses the PhotonVision camera object and adds in the simulated camera configuration to create a simuluated camera view. 
            m_cameraSim = new PhotonCameraSim(m_camera, cameraSettings);

            // Sets the position of the camera relative to the center of the robot
            // In this example, our camera is mounted 0.1 meters forward (x) and 0.5 meters up (y) from the robot pose
            // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
            Translation3d robotToCameraTrl = new Translation3d(0.1, 1, 0.5);

            // Sets the pitch and the rotation of the camera relative to the robot.
            // In this example, the camera is pitched 15 degrees up and rotated 0 degrees.
            Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);

            // Combines the translational and rotational data of the camera into one Transform3d object.
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

            // Adds the simulated camera (with its configuration) alongside the camera's position on the robot
            // This creates the camera pose on the robot, which, depending on how offsetted it is from the robot, can be seen in odometry via Glass (Robot Simulation)
            // Click on the NetworkTables tab -> SmartDashboard -> VisionSystemSim-main -> Sim Field
            m_visionSim.addCamera(m_cameraSim, robotToCamera);

            // Lets you see the simulated camera's view, which also shows which April Tags are seen in its view
            // To see this simulated view, on your broswer, go to: localhost:1182
            // NOTE: You must be currently simulating to be able to see this!
            m_cameraSim.enableDrawWireframe(true);
        }
    }

    public enum DetectionStatus {
        NONE,
        ONE,
        TWO,
        MULTIPLE,
        UNKNOWN
    }

    public DetectionStatus getDetectionStatus() {
        return detectionStatus;
    }

    public void setDetectionStatus(DetectionStatus status) {
        detectionStatus = status;
    }

    public void update() {
        for (var results : m_camera.getAllUnreadResults()) {
            PhotonTrackedTarget result = results.getBestTarget();
            if (result != null) {
                Transform3d aprilTagOffset = result.getBestCameraToTarget().plus(PhotonConstants.camToRobotOffset);
                aprilTagTx = aprilTagOffset.getMeasureX().in(Units.Meters);
                aprilTagTy = aprilTagOffset.getMeasureY().in(Units.Meters);
                aprilTagRot = aprilTagOffset.getRotation().getAngle();
                aprilTagID = result.fiducialId;
                aprilTagArea = result.getArea();
                detectedTagsCount = results.getTargets().size();

                switch (detectedTagsCount) {
                    case 1: 
                        setDetectionStatus(DetectionStatus.ONE);
                        break;
                    case 2: 
                        setDetectionStatus(DetectionStatus.TWO);
                        break;    
                    default: 
                        setDetectionStatus(DetectionStatus.MULTIPLE);
                        break;
                }
            }

            else {
                setDetectionStatus(DetectionStatus.NONE);
            }
        }
    }

    public double getTagTX() {
        return aprilTagTx;
    }

    public double getTagTY() {
        return aprilTagTy;
    }

    public double getTagRot() {
        return aprilTagRot;
    }

   /**
   * <p>This method updates the telemetry data on SmartDashboard.
   */
    public void updateDashboard() {
        SmartDashboard.putNumber("TAG TX", aprilTagTx);
        SmartDashboard.putNumber("TAG TY", aprilTagTy);
        SmartDashboard.putNumber("TAG ROT", aprilTagRot);
        SmartDashboard.putNumber("TAG ID", aprilTagID);
        SmartDashboard.putNumber("TAG AREA", aprilTagArea);
        SmartDashboard.putNumber("DETECTED TAGS", detectedTagsCount);
        SmartDashboard.putString("Detection Status", getDetectionStatus().toString());
    }

    @Override
    public void periodic() {
        // Updates the April Tag data (such as its offset).
        update();

        // Updates telemetry data onto SmartDashboard.
        updateDashboard();
    }


    @Override
    public void simulationPeriodic() {

        // Gets the robot pose
        Pose2d robotPose = m_driveSubsystem.getPose();

        // A Field2d object representing the robot and the April Tags on the field
        var debugField = m_visionSim.getDebugField();

        // Updates the robot pose in the Field2d Object
        debugField.getObject("EstimatedRobot").setPose(robotPose);

        // Updates the robot pose in the vision sim so that the camera's position is moving alongside the robot.
        m_visionSim.update(robotPose);
    }
}
