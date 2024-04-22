package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import edu.wpi.first.hal.DriverStationJNI;

public class TeamPropPipeline extends OpenCvPipeline {
    private final int width = Constants.VisionConstants.kWidth; // width of the image
    private final int height = Constants.VisionConstants.kHeight;

    private double leftPercent = 0, middlePercent = 0, rightPercent = 0;
    private final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0.33 * width, height)
    );
    private final Rect MIDDLE_RECTANGLE = new Rect(
            new Point(0.33 * width, 0),
            new Point(0.66 * width, height));
    private final Rect RIGHT_RECTANGLE= new Rect(
            new Point(0.66 * width, 0),
            new Point(width, height));

    private ConfigSubsystem mConfigSubsystem;

    public TeamPropPipeline(ConfigSubsystem configSubsystem){
        mConfigSubsystem = configSubsystem;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowRedLow = new Scalar(0, 100, 125);
        Scalar lowRedHigh = new Scalar(10, 255, 255);
        Scalar highRedLow = new Scalar(165, 100, 125);
        Scalar highRedHigh = new Scalar(180, 255, 255);

        Scalar blueLow = new Scalar(105, 50, 50);
        Scalar blueHigh = new Scalar(136, 255, 255);

        /*Mat redMat = new Mat();
        Core.bitwise_or(lowRedMat, highRedMat, redMat);

        Mat blueMat = new Mat();
        Core.inRange(mat, blueLow, blueHigh, blueMat);

        Mat redAndBlueMat = new Mat();
        Core.bitwise_or(redMat, blueMat, redAndBlueMat);*/
        Mat colorMat = new Mat();
        if (mConfigSubsystem.allianceColor() == Constants.ConfigConstants.kGauche) {
            Mat lowRedMat = new Mat();
            Core.inRange(mat, lowRedLow, lowRedHigh, lowRedMat);

            Mat highRedMat = new Mat();
            Core.inRange(mat, highRedLow, highRedHigh, highRedMat);
            Core.bitwise_or(lowRedMat, highRedMat, colorMat);
        }
        else {
            Core.inRange(mat, blueLow, blueHigh, colorMat);
        }

        leftPercent = Core.sumElems(colorMat.submat(LEFT_RECTANGLE)).val[0] / 255 / LEFT_RECTANGLE.area();
        middlePercent = Core.sumElems(colorMat.submat(MIDDLE_RECTANGLE)).val[0] / 255 / MIDDLE_RECTANGLE.area();
        rightPercent = Core.sumElems(colorMat.submat(RIGHT_RECTANGLE)).val[0] /255 / RIGHT_RECTANGLE.area();

        DriverStationJNI.getTelemetry().addData("leftPercent", leftPercent);
        DriverStationJNI.getTelemetry().addData("middlePercent", middlePercent);
        DriverStationJNI.getTelemetry().addData("rightPercent", rightPercent);

        return colorMat;

    }

    public int getTeamPropLocation() {
        double leftTreshold = 0.1;
        double rightTreshold = 0.1;
        double middleTreshold = 0.1;
        if (leftPercent > middlePercent && leftPercent > rightPercent && leftPercent > leftTreshold) {
            return 0;
        } else if (middlePercent > leftPercent && middlePercent > rightPercent && middlePercent > middleTreshold) {
            return 1;
        } else if (rightPercent > leftPercent && rightPercent > middlePercent && rightPercent > rightTreshold) {
            return 2;
        }
        return 0;
    }

}
