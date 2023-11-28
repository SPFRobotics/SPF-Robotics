package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.cameraDetectColorTest1;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous
public class AutoIntakeAiden extends LinearOpMode {
    //MOVEMENT MOTOR VARS
    private static final double strafeMult = 1.2;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private IMU imu = null;




    //INTAKE MOTOR VARS
    private DcMotor intake = null;

    //Color Vars
    OpenCvCamera camera;
    cameraDetectColorTest1 gameObjectDetection = new cameraDetectColorTest1();
    /*final*/ String spikeLocation = gameObjectDetection.getPosition().toString();


    //Outtake Vars
    Servo wristLeft = null;
    Servo wristRight = null;
    double wristPos = 0;
    double minWristPos = -1.0;
    double maxWristPos = 0.9;


    //April Tag
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int targetId = 0;


    //INTAKE FUNCTIONS
    public void initializeIntake(){
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void intake(double power, long sec) {
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(power);
        sleep(sec * 1000);
        intake.setPower(0);
    }
    public void placeOnSpikeMark(){
        //Move to center of spike marks
        //spikeLocation = "LEFT";
        double power = -.3;
        if(spikeLocation.equals("LEFT")) {
            move(.3, "forward", 18);
            move(.3, "left", 12);
            intake(power, 3);
            move(.3, "right", 12);
            move(.3, "backward", 18);
        } else if(spikeLocation.equals("RIGHT")){
            move(.3, "forward", 18);
            move(.3, "right", 12);
            intake(power, 3);
            move(.3, "left", 12);
            move(.3, "backward", 18);
        } else if(spikeLocation.equals("CENTER")){
            move(.3, "forward", 25);
            intake(power, 3);
            move(.3, "backward", 25);
        } else {
            telemetry.addData("Team Element", "Not Found");
            telemetry.update();
        }
    }



    //MOVEMENT FUNCTIONS
    //3.78(in inches, 9.6012 is centimeters) is the diameter of the wheel, and 537.7 is how many motor counts are in 1 full rotation of the motor's axle
    private double inch_convert(double inch) { return inch * (537.7 / (3.78 * Math.PI)); }
    private double inToCm(int inches) { return inches * 2.54; }
    private double cmToIn(double cm) { return cm / 2.54; }
    private double cm_convert(double cm) { return cm * (537.7 / (9.6012 / Math.PI)); }

    void parkFarRed(){
        move(.3, "forward", 5.5);
        move(.3, "right", 96);
    }
    void parkCloseRed(){
        move(.3, "forward", 3);
        move(.3, "right", 46);
    }
    void parkFarBlue(){
        move(.3, "forward", 5.5);
        move(.3, "left", 96);
    }
    void parkCloseBlue(){
        move(.3, "forward", 3);
        move(.3, "left", 46);
    }

    private void initializeMovement() {
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        stop_and_reset_encoders_all();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        //waitForStart();
    }
    private void stop_and_reset_encoders_all() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void run_to_position_all() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void powerZero() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
    private void move(double movePower, String moveDirection, double moveDistance){
        stop_and_reset_encoders_all(); //Sets encoder count to 0
        if (moveDirection.equals("forward")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance));
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
            telemetry.addData("Power", movePower);
            telemetry.update();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("backward")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("right")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("left")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(movePower);
        } else {
            telemetry.addData("Error", "move direction must be forward,backward,left, or right.");
            telemetry.update();
            terminateOpModeNow();
        }
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("test", "attempting to move...");
            telemetry.addData("power back right", backRight.getPower());
            telemetry.addData("power back left", backLeft.getPower());
            telemetry.addData("power front right", frontRight.getPower());
            telemetry.addData("power front left", frontLeft.getPower());
            telemetry.update();
        }
        powerZero();
        telemetry.addData("test", "done!");
        telemetry.update();
    }
    private void rotate(double angle, double power) {
        double Kp = 0.5; //this is for porposanal control (ie. the closer you are the target angle the slower you will go)
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetAngle = startAngle + angle;
        double error = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
        double power1 = power;
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rotate until the target angle is reached
        System.out.printf("%f start angle = ",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        System.out.printf("%f error = ", error);
        while (opModeIsActive() && Math.abs(error) > 5) {
            //powerZero();
            error = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            // the closer the robot is to the target angle, the slower it rotates
            //power = Range.clip(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle) / 90, 0.1, 0.5);
            power1 = power * error * Kp;
            power1 = Range.clip(power1,-0.5,0.5); //"Range.clip(value, minium, maxium)" takes the first term and puts it in range of the min and max provided
            telemetry.addData("power1",power1);
            System.out.printf("%f power = ",power1);
            telemetry.addData("error",error);
            telemetry.addData("power", power);
            telemetry.addData("Kp",Kp);

            backLeft.setPower(power1);
            backRight.setPower(-power1);
            frontLeft.setPower(power1);
            frontRight.setPower(-power1);
            if (Math.abs(error) <= 1) {
                powerZero();
            }
            telemetry.addData("angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("target", targetAngle);
            telemetry.update();
            //double angleDifference = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - targetAngle);
            //rotate(angleDifference, power);
        }
        powerZero();
    }


    //Claw funcs
    public void initializeOuttake(){
        intake = hardwareMap.dcMotor.get("intake");  /** Port: ExpansionHub MotorPort 1 **/
        wristLeft = hardwareMap.servo.get("wristLeft"); /** Port: ExpansionHub ServoPort 4 **/
        wristRight = hardwareMap.servo.get("wristRight");
        wristLeft.setDirection(Servo.Direction.REVERSE);
    }
    public void outtake(double targetPos){
        wristPos = Range.clip(targetPos,minWristPos,maxWristPos);
        wristLeft.setPosition(targetPos);
        wristRight.setPosition(targetPos);
    }


    //April Tag Functions
    private void initializeAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public AprilTagDetection checkCenter(){
        //Resolution Camera == 320 | 160
        //aprilTag.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(int i = 0; i<currentDetections.size(); i++){
            AprilTagDetection detection = currentDetections.get(i);
            if(135 < detection.center.x && detection.center.x < 185){
                return currentDetections.get(i);
            }
        }
        return null;
    }
    private void theGoat(){
        AprilTagDetection detection = checkCenter();
        if(detection.id == targetId){

        }
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


    //Run Op Mode
    public void runOpMode(){
        initializeIntake();
        initializeMovement();
        initializeOuttake();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        gameObjectDetection = new cameraDetectColorTest1();
        camera.setPipeline(gameObjectDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("problem"," error");
            }
        });

        while(!isStarted()){
            spikeLocation = gameObjectDetection.getPosition().toString();
            telemetry.addData("Test", "working");
            telemetry.addData("Location", spikeLocation);
            telemetry.update();
        }
        waitForStart();
        if(opModeIsActive()) {
            placeOnSpikeMark();
            parkFarRed(); //Park method based on position.  Far means far from backdrop, close means close to backdrop
        }
    }

    MecanumChassis mc = new MecanumChassis(this);







































}
