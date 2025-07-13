package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

public class PhotonSubsystem extends SubsystemBase {

    private DriveSubsystem m_driveSubsystem;
    private PhotonCamera m_camera;
    private Transform3d camToRobotTsf;
    private DetectedTags detectionStatus = DetectedTags.NONE;
    private String cameraName;

    private double aprilTagRot = 0;
    private double aprilTagTx = 0;
    private double aprilTagTy = 0;
    private double aprilTagArea = 0;
    private int aprilTagID = 0;
    private int detectedTagsCount = 0;


    public PhotonSubsystem(DriveSubsystem m_driveSubsystem, CameraProperties m_camProperties) {
        
        // Creates the PhotonVision camera object. This is the non-sim object. In simulation, it is just used as a reference.
        // Be sure the camera name matches the name set in PhotonVision. Otherwise, it will not detect the camera.
        cameraName = m_camProperties.getCameraName();
        m_camera = new PhotonCamera(cameraName);
        camToRobotTsf = m_camProperties.getCamToRobotTsf();
    }

    public enum DetectedTags {
        NONE,
        ONE,
        TWO,
        MULTIPLE,
        UNKNOWN
    }

    public DetectedTags getDetectionStatus() {
        return detectionStatus;
    }

    public void setDetectionStatus(DetectedTags status) {
        detectionStatus = status;
    }

    public PhotonCamera getCamera() {
        return m_camera;
    }

    public Transform3d getCamToRobotTsf() {
        return camToRobotTsf;
    }

    public void update() {
        for (var results : m_camera.getAllUnreadResults()) {
            PhotonTrackedTarget result = results.getBestTarget();
            if (result != null) {
                Transform3d aprilTagOffset = result.getBestCameraToTarget().plus(camToRobotTsf);
                aprilTagTx = aprilTagOffset.getMeasureX().in(Units.Meters);
                aprilTagTy = aprilTagOffset.getMeasureY().in(Units.Meters);
                aprilTagRot = aprilTagOffset.getRotation().getAngle();
                aprilTagID = result.fiducialId;
                aprilTagArea = result.getArea();
                detectedTagsCount = results.getTargets().size();

                switch (detectedTagsCount) {
                    case 1: 
                        setDetectionStatus(DetectedTags.ONE);
                        break;
                    case 2: 
                        setDetectionStatus(DetectedTags.TWO);
                        break;    
                    default: 
                        setDetectionStatus(DetectedTags.MULTIPLE);
                        break;
                }
            }

            else {
                setDetectionStatus(DetectedTags.NONE);
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

    public int getTagID() {
        return aprilTagID;
    }

    public double getTagArea() {
        return aprilTagArea;
    }

   /**
   * <p>This method updates the telemetry data on SmartDashboard.
   */
    public void updateDashboard() {
        SmartDashboard.putNumber(cameraName + " TAG TX", aprilTagTx);
        SmartDashboard.putNumber(cameraName + " TAG TY", aprilTagTy);
        SmartDashboard.putNumber(cameraName + " TAG ROT", aprilTagRot);
        SmartDashboard.putNumber(cameraName + " TAG ID", aprilTagID);
        SmartDashboard.putNumber(cameraName + " TAG AREA", aprilTagArea);
        SmartDashboard.putNumber(cameraName + " DETECTED TAGS", detectedTagsCount);
        SmartDashboard.putString(cameraName + " Detection Status", getDetectionStatus().toString());
    }

    @Override
    public void periodic() {
        // Updates the April Tag data (such as its offset).
        update();

        // Updates telemetry data onto SmartDashboard.
        updateDashboard();
    }

}
