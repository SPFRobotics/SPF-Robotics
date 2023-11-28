package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class cameraDetectColorTest1 extends OpenCvPipeline {

    public enum GameObjectLocation {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
    private static Point GameObject_BoundingBox_TopLeft_AnchorPoint = new Point(0,120);
    private static int BoundingBox_Width = 320;
    private static int BoundingBox_Height = 80;

    private static int LeftLineLocation = 60; //From the left of bounding box
    private static int CenterLeftLineLocation = 120;
    private static int CenterRightLineLocation = 200;
    private static int RightLineLocation = 260; //From the left of bounding box

    private static final Scalar
        lower_red_bounds  = new Scalar(100,0,0,0),
        upper_red_bounds  = new Scalar(255,70,70,255),
        lower_blue_bounds = new Scalar(0,0,100,0),
        upper_blue_bounds = new Scalar(100,100,255,255);

    private final Scalar
        RED   = new Scalar(255,0,0),
        BLUE  = new Scalar(0,0,255),
        BLACK = new Scalar(0,0,0);
    private Scalar color = BLACK;

    public double minSectorPercent = 144, minTotalPercent = 1040;
    public double maxPercent = 0, highestSector = 0;
    public double redPercent = 0, bluePercent = 0;
    private Mat redMat = new Mat(), blueMat = new Mat(), blurredMat = new Mat();
    public double redPercentLeft, bluePercentLeft, leftPercent = 0;
    private Mat redMatLeft = new Mat(), blueMatLeft = new Mat(), blurredMatLeft = new Mat();
    public double redPercentCenter, bluePercentCenter, centerPercent = 0;
    private Mat redMatCenter = new Mat(), blueMatCenter = new Mat(), blurredMatCenter = new Mat();
    //public double redPercentRight, bluePercentRight, rightPercent = 0;
    //private Mat redMatRight = new Mat(), blueMatRight = new Mat(), blurredMatRight = new Mat();

    Point GameObjectPointA = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y);
    Point GameObjectPointB = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + BoundingBox_Width,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y + BoundingBox_Height);
    Point GameObjectLeftPointA = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y);
    Point GameObjectLeftPointB = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + LeftLineLocation,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y + BoundingBox_Height);
    Point GameObjectCenterPointA = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + CenterLeftLineLocation,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y);
    Point GameObjectCenterPointB = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + CenterRightLineLocation,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y + BoundingBox_Height);
    Point GameObjectRightPointA = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + RightLineLocation,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y);
    Point GameObjectRightPointB = new Point(
            GameObject_BoundingBox_TopLeft_AnchorPoint.x + BoundingBox_Width,
            GameObject_BoundingBox_TopLeft_AnchorPoint.y + BoundingBox_Height);

    private volatile GameObjectLocation position = GameObjectLocation.NONE;

    @Override
    public Mat processFrame(Mat input){
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        Imgproc.blur(input, blurredMatLeft, new Size(5, 5));
        Imgproc.blur(input, blurredMatCenter, new Size(5, 5));
        //Imgproc.blur(input, blurredMatRight, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(GameObjectPointA, GameObjectPointB));
        blurredMatLeft = blurredMatLeft.submat(new Rect(GameObjectLeftPointA, GameObjectLeftPointB));
        blurredMatCenter = blurredMatCenter.submat(new Rect(GameObjectCenterPointA, GameObjectCenterPointB));
        //blurredMatRight = blurredMatRight.submat(new Rect(GameObjectRightPointA, GameObjectRightPointB));


        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blurredMatLeft, blurredMatLeft, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(blurredMatCenter, blurredMatCenter, Imgproc.MORPH_CLOSE, kernel);
        //Imgproc.morphologyEx(blurredMatRight, blurredMatRight, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, blueMat);
        Core.inRange(blurredMatLeft, lower_red_bounds, upper_red_bounds, redMatLeft);
        Core.inRange(blurredMatLeft, lower_blue_bounds, upper_blue_bounds, blueMatLeft);
        Core.inRange(blurredMatCenter, lower_red_bounds, upper_red_bounds, redMatCenter);
        Core.inRange(blurredMatCenter, lower_blue_bounds, upper_blue_bounds, blueMatCenter);
        /*Core.inRange(blurredMatRight, lower_red_bounds, upper_red_bounds, redMatRight);
        Core.inRange(blurredMatRight, lower_blue_bounds, upper_blue_bounds, blueMatRight);*/

        // Gets color specific values
        redPercent = Core.countNonZero(redMat);
        bluePercent = Core.countNonZero(blueMat);
        redPercentLeft = Core.countNonZero(redMatLeft);
        bluePercentLeft = Core.countNonZero(blueMatLeft);
        leftPercent = redPercentLeft + bluePercentLeft;
        redPercentCenter = Core.countNonZero(redMatCenter);
        bluePercentCenter = Core.countNonZero(blueMatCenter);
        centerPercent = redPercentCenter + bluePercentCenter;
        /*redPercentRight = Core.countNonZero(redMatRight);
        bluePercentRight = Core.countNonZero(blueMatRight);
        rightPercent = redPercentRight + bluePercentRight;*/

        // Calculates the highest amount of pixels being covered on each side
        maxPercent = Math.max(Math.max(redPercent, bluePercent),minTotalPercent);
        //double highestSector = Math.max(Math.max(leftPercent,centerPercent),rightPercent);
        highestSector = Math.max(Math.max(leftPercent,centerPercent),minSectorPercent);

        //maxPercent = 0;

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (highestSector != minSectorPercent && maxPercent != minTotalPercent) {
            if (maxPercent == redPercent) {
                color = RED;
            } else if (maxPercent == bluePercent) {
                color = BLUE;
            } else {
                color = BLACK;
            }
            if (highestSector == leftPercent) {
                position = GameObjectLocation.LEFT;
                Imgproc.rectangle(input,GameObjectLeftPointA,GameObjectLeftPointB,color,4);
            } else if (highestSector == centerPercent) {
                position = GameObjectLocation.CENTER;
                Imgproc.rectangle(input, GameObjectCenterPointA, GameObjectCenterPointB, color, 4);
            } else {
                position = GameObjectLocation.RIGHT;
                Imgproc.rectangle(input,GameObjectRightPointA,GameObjectRightPointB,color,4);
            }
            /*} else if (highestSector == rightPercent) {
                position = GameObjectLocation.RIGHT;
                Imgproc.rectangle(input,GameObjectRightPointA,GameObjectRightPointB,color,4);
            }*/
        } else if (maxPercent == 0 || maxPercent == minTotalPercent) {
            //position = GameObjectLocation.NONE;
            position = GameObjectLocation.RIGHT;
            //Imgproc.rectangle(input,GameObjectPointA,GameObjectPointB,BLACK,4);
            Imgproc.rectangle(input,GameObjectLeftPointA,GameObjectLeftPointB,BLACK,2);
            Imgproc.rectangle(input,GameObjectCenterPointA,GameObjectCenterPointB,BLACK,2);
            Imgproc.rectangle(input,GameObjectRightPointA,GameObjectRightPointB,BLACK,2);
        }

        // Memory cleanup
        blurredMat.release();
        blurredMatLeft.release();
        blurredMatCenter.release();
        //blurredMatRight.release();
        redMat.release();
        redMatLeft.release();
        redMatCenter.release();
        //redMatRight.release();
        blueMat.release();
        blueMatLeft.release();
        blueMatCenter.release();
        //blueMatRight.release();

        return input;
    }

    public GameObjectLocation getPosition() {
        return position;
    }
    public double[] randomStuffInfo() {
        double[] info = {maxPercent,highestSector};
        return info;
    }
}
