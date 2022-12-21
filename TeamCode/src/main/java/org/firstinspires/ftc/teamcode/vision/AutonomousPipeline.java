package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutonomousPipeline extends OpenCvPipeline {

    private VisionSubsystem mVisionSubsystem;
    public AutonomousPipeline(VisionSubsystem visionSubsystem) {
        mVisionSubsystem = visionSubsystem;



    }
    @Override
    public Mat processFrame(Mat input) {
        mVisionSubsystem.setAutonomousPosition(1);
        return null;
    }
}
