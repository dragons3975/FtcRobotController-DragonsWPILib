package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class VisionSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private OpenCvCamera mVuforiaPassThroughCam;
     AprilTagDetectionPipeline mAprilTagDetectionPipeline;

     private boolean isVisionEnabled = false;

    private int mAutonomousPosition = 0;


    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        mVuforiaPassThroughCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        mAprilTagDetectionPipeline = new AprilTagDetectionPipeline(Constants.VisionConstants.ktagsize, Constants.VisionConstants.kfx, Constants.VisionConstants.kfy, Constants.VisionConstants.kcx, Constants.VisionConstants.kcy);

        mVuforiaPassThroughCam.setPipeline(mAprilTagDetectionPipeline);
        mVuforiaPassThroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                mVuforiaPassThroughCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    @Override
    public void periodic() {

        if (isVisionEnabled == true) {
            ArrayList<AprilTagDetection> currentDetections = mAprilTagDetectionPipeline.getLatestDetections();
            mTelemetry.addData("List Size", currentDetections.size());
            if(currentDetections.size() != 0) {
                mAutonomousPosition = currentDetections.get(currentDetections.size()-1).id;
            }
            mTelemetry.addData("Autonomous Position", mAutonomousPosition);
        }
    }

    public void enableVision() {
        isVisionEnabled = true;
    }
    public void disableVision() {
        isVisionEnabled = false;
    }

    public int getAutonomousPosition() {
        return mAutonomousPosition;
    }

}
