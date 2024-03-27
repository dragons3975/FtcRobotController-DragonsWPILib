package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    OpenCvCamera webcam;

    public VisionSubsystem() {
        int cameraMonitorViewId = DriverStationJNI.getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", DriverStationJNI.getHardwareMap().appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(mTeamPropPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }
    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("teamPropLocation", getTeamPropLocation());
    }

    public int getTeamPropLocation() {
        return mTeamPropPipeline.getTeamPropLocation();
    }

    public String getTeamPropLocationToString() {
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