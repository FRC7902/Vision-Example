package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraProperties {

    private final Transform3d camToRobotTsf;
    private final String camName;

    public CameraProperties(String camName, Transform3d camToRobotTsf) {
        this.camName = camName;
        this.camToRobotTsf = camToRobotTsf;
    }

    public Transform3d getCamToRobotTsf() {
        return camToRobotTsf;
    }

    public String getCameraName() {
        return camName;
    }
    
}
