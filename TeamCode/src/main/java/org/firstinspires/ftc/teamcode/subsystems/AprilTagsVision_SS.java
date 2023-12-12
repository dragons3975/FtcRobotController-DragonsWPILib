package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;
import dragons.rev.FtcMotorSimple;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AprilTagsVision_SS extends Subsystem {

    AprilTagProcessor m_AprilTagProcessor;
    VisionPortal m_VisionPortal;


    public AprilTagsVision_SS() {
        m_AprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        m_VisionPortal= new VisionPortal.Builder()
                .setCamera(DriverStationJNI.getHardwareMap().get(WebcamName.class,"ALCamera"))
               //.setCamera(hardwareMap.get(WebcamName.class, "ALCamera"))
                .addProcessor(m_AprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }

    @Override
    public void periodic() {
      //  DriverStationJNI.getTelemetry().addData("distanceX", 0);

       // DriverStationJNI.getTelemetry().addData("mGyro angle", 0);
        List<AprilTagDetection> currentDetections = m_AprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();

        for (AprilTagDetection detection : currentDetections){

            idsFound.append(detection.id);
            idsFound.append("");
        }
        DriverStationJNI.getTelemetry().addData("AprilTags", idsFound);
    }



}



