package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.AutonomousPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraListener;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionSubsystem extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;
    private DriveSubsystem mDriveSubsystem;
    private OpenCvCamera mVuforiaPassThroughCam;
    private VuforiaLocalizer mVuforiaLocalizer;
    private int mAutonomousPosition = 0;


    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);

        parameters.vuforiaLicenseKey = Constants.VuforiaConstants.kVuforiaKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //mVuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        //mVuforiaPassThroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(mVuforiaLocalizer, parameters, viewportContainerIds[1]);

        //AutonomousPipeline mAutonomousPipeline = new AutonomousPipeline(this);
        //CameraListener listener = new CameraListener(mVuforiaPassThroughCam, mAutonomousPipeline);
        //mVuforiaPassThroughCam.openCameraDeviceAsync(listener);
    }

    @Override
    public void periodic() {
        mTelemetry.addData("Autonomous Position", mAutonomousPosition);
    }
    public void setAutonomousPosition(int position) {
        mAutonomousPosition = position;
    }
    public int getAutonomousPosition() {
        return mAutonomousPosition;
    }


}


