package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionSubsystem extends Subsystem {
    
    private ConfigSubsystem mConfigSubsystem;
    //private AprilTagProcessor mAprilTagProcessor;
    private TfodProcessor mTfodProcessor;
    private VisionPortal mVisionPortal;

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue",
            "red"
    };

    private double mTeamPropX;
    private boolean mIsRedDetected = false;
    private boolean mIsBlueDetected = false;

    public VisionSubsystem(ConfigSubsystem configSubsystem)
    {
        mConfigSubsystem = configSubsystem;

        //mAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        mTfodProcessor = new TfodProcessor.Builder()
                .setModelFileName(Constants.VisionConstants.kTfodModelFile)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(mTfodProcessor);
        mVisionPortal = builder.build();
    }

    @Override
    public void periodic() {
        
        detectTeamPropLocation();

        DriverStationJNI.getTelemetry().addData("isRedDetected", mIsRedDetected);
        DriverStationJNI.getTelemetry().addData("isBlueDetected", mIsBlueDetected);
        DriverStationJNI.getTelemetry().addData("mTeamPropX", mTeamPropX);
        DriverStationJNI.getTelemetry().addData("Team Prop Location", getTeamPropLocationToString());

    }

    private void detectTeamPropLocation() {
        List<Recognition> currentRecognitions = mTfodProcessor.getRecognitions();

        mIsRedDetected = false;
        mIsBlueDetected = false;

        for (Recognition recognition : currentRecognitions) {
            if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kRouge && recognition.getLabel() == "red") {
                mIsRedDetected = true;
                mTeamPropX = (recognition.getLeft() + recognition.getRight()) / 2 ;
            }
            if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kBleu && recognition.getLabel() == "blue") {
                mIsBlueDetected = true;
                mTeamPropX = (recognition.getLeft() + recognition.getRight()) / 2 ;
            }
        }

        /*List<AprilTagDetection> currentDetections = mAprilTagProcessor.getDetections();
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
        }*/

    }
    /*public void turnOffCamera() {
        mCamera.close();
    }
    public void stopCamera() {
        mCamera.stopStreaming();
    }
    public void resumeCamera() {
        mCamera.resumeStreaming();
    }*/

    public int getTeamPropLocation() {
        return mConfigSubsystem.temporaryTeamPropLocation(); //temporaire tant que pas de vision
        /*if (mTeamPropX < Constants.VisionConstants.kLimiteG) {
            return Constants.VisionConstants.kTeamPropGauche;
        }
        if (mTeamPropX > Constants.VisionConstants.kLimiteG && mTeamPropX < Constants.VisionConstants.kLimiteD) {
            return Constants.VisionConstants.kTeamPropMilieu;
        }
        return Constants.VisionConstants.kTeamPropDroite;*/
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
