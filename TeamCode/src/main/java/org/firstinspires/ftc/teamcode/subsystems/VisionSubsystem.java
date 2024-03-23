package org.wolfcorp.skystone;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TeamPropPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionSubsystem extends Subsystem {
    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    TeamPropPipeline mTeamPropPipeline = new TeamPropPipeline();
    OpenCvCamera phoneCam;

    @Override
    public void periodic() {
        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = DriverStationJNI.getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", DriverStationJNI.getHardwareMap().appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        phoneCam.setPipeline(mTeamPropPipeline);
        // Remember to change the camera rotation
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);
        DriverStationJNI.getTelemetry().addData("teamPropLocation", getTeamPropLocation());
    }

    public int getTeamPropLocation() {
        return mTeamPropPipeline.getTeamPropLocation();
    }

    private String getTeamPropLocationToString() {
        switch (getTeamPropLocation()) {
            case Constants.VisionConstants.kTeamPropGauche:
                return "gauche";
            case Constants.VisionConstants.kTeamPropMilieu:
                return "milieu";
            case Constants.VisionConstants.kTeamPropDroite:
                return "droite";
            default:
                return "null";
        }
    }
}