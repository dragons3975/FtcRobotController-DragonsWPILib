package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class CameraSubsystem extends Subsystem {
    private VisionPortal camera;

    private AprilTagProcessor aprilTag;

    private boolean active = true;
    
    public CameraSubsystem() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        camera = camera.easyCreateWithDefaults(DriverStationJNI.getHardwareMap().get(WebcamName.class, "Webcam 1"), aprilTag);
    }
    

    @Override
    public void periodic() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

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
        }

        DriverStationJNI.getTelemetry().addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        DriverStationJNI.getTelemetry().addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        DriverStationJNI.getTelemetry().addLine("RBE = Range, Bearing & Elevation");
    }

    public void areteCamera() {
        active = false;
        camera.stopStreaming();
    }

    public void activerCamera() {
        active = true;
        camera.resumeStreaming();
    }

    public void toggle() {
        if (active) {
            areteCamera();
        } else {
            activerCamera();
        }
    }


}

