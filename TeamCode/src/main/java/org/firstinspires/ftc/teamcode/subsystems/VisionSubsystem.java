package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TeamPropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.sql.Driver;
import java.util.List;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionSubsystem extends Subsystem {

    private boolean mIsPipelineProp = true;
    private AprilTagProcessor mAprilTag;
    private VisionPortal mVisionPortal;

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    TeamPropPipeline mTeamPropPipeline = new TeamPropPipeline();
    ConceptAprilTagEasy mConceptAprilTagEasy = new ConceptAprilTagEasy();
    OpenCvCamera webcam;




    public VisionSubsystem() {
        int cameraMonitorViewId = DriverStationJNI.getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", DriverStationJNI.getHardwareMap().appContext.getPackageName());

        if (mIsPipelineProp) {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(mTeamPropPipeline);
        }
        else {
            mVisionPortal = VisionPortal.easyCreateWithDefaults(DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"), mAprilTag);
        }

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                if (mIsPipelineProp) {
               webcam.startStreaming(Constants.VisionConstants.kWidth, Constants.VisionConstants.kHeight, OpenCvCameraRotation.UPRIGHT);
                }
                else {webcam.stopStreaming();
                mVisionPortal.resumeStreaming();}
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
        if (!mIsPipelineProp) {telemetryAprilTag();}

        DriverStationJNI.getTelemetry().addData("IsPipeline", mIsPipelineProp);
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
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = mAprilTag.getDetections();
        DriverStationJNI.getTelemetry().addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                DriverStationJNI.getTelemetry().addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                DriverStationJNI.getTelemetry().addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                DriverStationJNI.getTelemetry().addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                DriverStationJNI.getTelemetry().addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                DriverStationJNI.getTelemetry().addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                DriverStationJNI.getTelemetry().addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        DriverStationJNI.getTelemetry().addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        DriverStationJNI.getTelemetry().addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        DriverStationJNI.getTelemetry().addLine("RBE = Range, Bearing & Elevation");

    }

    public void togglePipeline() {
        mIsPipelineProp = !mIsPipelineProp;
    }
}