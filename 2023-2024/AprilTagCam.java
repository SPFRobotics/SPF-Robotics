package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagCam {
    public LinearOpMode opmode = null;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public int targetId = 0;

    public AprilTagCam(LinearOpMode lom){
        opmode = lom;

        //Define class variables in constructor instead of initialization
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    public AprilTagDetection checkCenter(){ //Might make error cuz return null value
        //Resolution Camera == 320p x 240p | 160

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(int i = 0; i<currentDetections.size(); i++){
            AprilTagDetection detection = currentDetections.get(i);
            if(135 < detection.center.x && detection.center.x < 185){
                return currentDetections.get(i);
            }
        }
        return null;
    }
    private void setTargetId(String location, String color){
        if(location.equals("LEFT")){
            targetId = 1;
        } else if(location.equals("CENTER")){
            targetId = 2;
        } else if(location.equals("RIGHT")){
            targetId = 3;
        }
        if(color.equals("RED")){
            targetId += 3;
        }
    }
}
