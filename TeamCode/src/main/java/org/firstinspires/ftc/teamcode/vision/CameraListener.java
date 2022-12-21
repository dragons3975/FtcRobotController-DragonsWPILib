package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraListener implements OpenCvCamera.AsyncCameraOpenListener {
    private OpenCvCamera mVuforiaPassThroughCam;
    private OpenCvPipeline mOpenCvPipeline;


    public CameraListener(OpenCvCamera openCvCamera, OpenCvPipeline openCvPipeline) {
        mVuforiaPassThroughCam = openCvCamera;
        mOpenCvPipeline = openCvPipeline;
    }


    @Override
    public void onOpened() {
        mVuforiaPassThroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        mVuforiaPassThroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        mVuforiaPassThroughCam.setPipeline(mOpenCvPipeline);

        mVuforiaPassThroughCam.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}
