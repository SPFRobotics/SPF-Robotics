/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Camera {
    String src; //File Location
    double resolution; //Camera Resolution L/W X/Y
    double FOV; //Field of View in Degrees
    TfodProcessor tfod;

    public Camera(String source, double res, double tempFov){
        src = source;
        resolution = res;
        FOV = tempFov;
    }

    //return true if tensor flow recognition
    boolean foundObject(){
        //To Do Done
        return false;
    }
    //Return 1,2,3 left to right
    int whichSpikeMark(){
        return 0;
    }
    //Return True If Sees April Tag
    boolean findAprilTag(){
        return false;
    }

    private void initTfod() {
        telemetry.addData("Started","Init");
        telemetry.update();
        // Create the TensorFlow processor by using a builder.
        src = "lite-model_ssd_mobilenet_v1_1_metadata_2.tflite";
        //path = "ssd_mobilenet_v2_fpnlite_640x640_coco17_tpu_8.tflite";

        tfod = new TfodProcessor.Builder()
                .setModelFileName(src)
                .setModelLabels(LABELS)
                .setModelInputSize(999)
                .setModelAspectRatio(resolution)
                .build();

        // With the following lines commented out, the default TfodProcessor Builder
        // will load the default model for the season. To define a custom model to load,
        // choose one of the following:
        //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
        //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
        //.setModelAssetName(TFOD_MODEL_ASSET)


        // The following default settings are available to un-comment and edit as needed to
        // set parameters for custom models.
        //.setModelLabels(LABELS)
        //.setIsModelTensorFlow2(true)
        //.setIsModelQuantized(true)
        //.setModelInputSize(300)
        //.setModelAspectRatio(16.0 / 9.0)

        //.build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
        telemetry.addData("Status:","Init Complete");
        telemetry.update();

    }   // end method initTfod()

}
*/