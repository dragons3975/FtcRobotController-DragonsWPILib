package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PixelDetectionSubsystem extends Subsystem {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal mCamera;
   // private TfodProcessor tfod = TfodProcessor.easyCreateWithDefaults();


    private double x, y;
    private boolean isDetected = false;

    public PixelDetectionSubsystem() {
        initAprilTag();
    }

    @Override
    public void periodic() {
        System.out.println(getPixelX());
        telemetryAprilTag();
        // Wait for the DS start button to be touched.
        DriverStationJNI.getTelemetry().addData("isDetected", isDetected);
        if (!tfod.getRecognitions().isEmpty()) {
            isDetected = true;
        }
    }
    private void initAprilTag() {
        tfod = TfodProcessor.easyCreateWithDefaults();
        tfod.setZoom(1.0);

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            mCamera = VisionPortal.easyCreateWithDefaults(
                    DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            mCamera = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }
    private void telemetryAprilTag() {
        // Add telemetry about TensorFlow Object Detection (TFOD) recognition
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        DriverStationJNI.getTelemetry().addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            DriverStationJNI.getTelemetry().addData(""," ");
            DriverStationJNI.getTelemetry().addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            DriverStationJNI.getTelemetry().addData("- Position", "%.0f / %.0f", x, y);
            DriverStationJNI.getTelemetry().addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
    public void turnOffCamera() {
        mCamera.close();
    }
    public void stopCamera() {
        mCamera.stopStreaming();
    }
    public void resumeCamera() {
        mCamera.resumeStreaming();
    }

    public double getPixelX() {
        return x;
    }
    public void setAutonomous() {

    }

    //public void ModifInclinaison(double pos) {
    //    mMotorPince.setPosition(pos);
    //}




}



