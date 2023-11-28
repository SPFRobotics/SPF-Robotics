package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ColorCam {
    LinearOpMode opmode = null;
    OpenCvCamera camera = null;
    cameraDetectColorTest1 gameObjectDetection = new cameraDetectColorTest1();
    String spikeLocation = gameObjectDetection.getPosition().toString();
    public ColorCam(LinearOpMode lom){
        opmode = lom;
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        gameObjectDetection = new cameraDetectColorTest1();
        camera.setPipeline(gameObjectDetection);
    }
    public void camOn(){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opmode.telemetry.addData("problem"," error");
            }
        });
    }
    public void camOff(){
        //camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
        //    @Override
        //    public void onClose() {
                camera.stopStreaming();
        //    }
        //});
    }
}
