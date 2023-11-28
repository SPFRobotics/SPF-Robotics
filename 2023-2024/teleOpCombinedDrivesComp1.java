package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Arrays;

@TeleOp
@Disabled
public class teleOpCombinedDrivesComp1 extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor intake;
    private Servo wristLeft;
    private Servo wristRight;
    private Servo clawLeft;
    private Servo clawRight;
    private IMU imu;
    private double speed1 = 0.5;
    private double speed1Default = 0.7;
    private double speed2 = 0.5;
    private double speed2Default = 0.7;
    private double liftSpeed = 1.0;
    private double servoSpeed = 0;
    private static final int liftMaxMotorCounts = 4062;
    private static final double minWristPos = -1.0;
    private static final double maxWristPos = 0.9;
    private static final double minClawPos = 0.7;
    private static final double maxClawPos = 0.5;
    private double previousSpeed1;
    private double previousSpeed2;
    private int iterationsPressed1 = 0;
    private int iterationsPressed2 = 0;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;
    private ElapsedTime timeRightTriggerHeld1 = new ElapsedTime();
    private ElapsedTime timeRightTriggerHeld2 = new ElapsedTime();
    private double timeHeldToConfirm = 0.5;
    int maxSpeedDuration = 50;
    private double maxSpeedRange1 = 1.0;
    private double maxSpeedRange2 = 1.0;
    private int liftPosition = 0;
    private double wristPos = 0.5;
    private boolean clawLeftToggle = false;
    private boolean clawRightToggle = false;
    private Quaternion currentIMUAngle;
    private Gamepad.RumbleEffect maxSpeedStartUpRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.1,0.1,maxSpeedDuration)
            .addStep(0.2,0.2,maxSpeedDuration)
            .addStep(0.3,0.3,maxSpeedDuration)
            .addStep(0.4,0.4,maxSpeedDuration)
            .addStep(0.5,0.5,maxSpeedDuration)
            .addStep(0.6,0.6,maxSpeedDuration)
            .addStep(0.7,0.7,maxSpeedDuration)
            .addStep(0.8,0.8,maxSpeedDuration)
            .addStep(0.9,0.9,maxSpeedDuration)
            .addStep(1.0,1.0,(maxSpeedDuration*2))
            .build();
    private Gamepad.LedEffect maxSpeedStartUpLEDEffect = new Gamepad.LedEffect.Builder()
            .addStep(0.1,0,0,maxSpeedDuration)
            .addStep(0.2,0,0,maxSpeedDuration)
            .addStep(0.3,0,0,maxSpeedDuration)
            .addStep(0.4,0,0,maxSpeedDuration)
            .addStep(0.5,0,0,maxSpeedDuration)
            .addStep(0.6,0,0,maxSpeedDuration)
            .addStep(0.7,0,0,maxSpeedDuration)
            .addStep(0.8,0,0,maxSpeedDuration)
            .addStep(0.9,0,0,maxSpeedDuration)
            .addStep(1.0,0,0,(maxSpeedDuration*2))
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        Initialization();

        if (isStopRequested()) return;

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        while (opModeIsActive()) {
            /*if (currentIMUAngle == imu.getRobotOrientationAsQuaternion() || currentIMUAngle == null) {
                imu = hardwareMap.get(IMU.class, "imu");
                // Adjust the orientation parameters to match your robot
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
                // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
                imu.initialize(parameters);
                telemetry.addData("worked","");
            }*/
            currentIMUAngle = imu.getRobotOrientationAsQuaternion();
            //telemetry.addData("imu connected: ", imu.getConnectionInfo());
            //telemetry.addData("imu hardware: ", currentIMUAngle);
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            Speed();
            if (Math.abs(gamepad1.left_stick_x) <= 0.05 && Math.abs(gamepad1.left_stick_y) <= 0.05) {
                fieldOriented();
            } else {
                robotOriented();
            }
            Intake();
            Outtake();
            LiftHold();
            //telemetry.addData("wristPos: ","Left: %f, Right: %f",wristLeft.getPosition(),wristRight.getPosition());
            //telemetry.addData("wristPos: ",wristPos);
            telemetry.update();
        }
    }

    private void Initialization() {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); /** Port: ControlHub MotorPort 0 **/
        backLeft = hardwareMap.dcMotor.get("backLeft"); /** Port: ControlHub MotorPort 2 **/
        frontRight = hardwareMap.dcMotor.get("frontRight"); /** Port: ControlHub MotorPort 1 **/
        backRight = hardwareMap.dcMotor.get("backRight"); /** Port: ControlHub MotorPort 3 **/
        liftLeft = hardwareMap.dcMotor.get("liftLeft"); /** Port: ExpansionHub MotorPort 3 **/
        liftRight = hardwareMap.dcMotor.get("liftRight"); /** Port: ExpansionHub MotorPort 2 **/
        intake = hardwareMap.dcMotor.get("intake");  /** Port: ExpansionHub MotorPort 1 **/
        wristLeft = hardwareMap.servo.get("wristLeft"); /** Port: ExpansionHub ServoPort 3 **/
        wristRight = hardwareMap.servo.get("wristRight"); /** Port: ExpansionHub ServoPort 5 **/
        clawLeft = hardwareMap.servo.get("clawLeft"); /** Port: ControlHub Servo Port 5 **/
        clawRight = hardwareMap.servo.get("clawRight"); /** Port: ControlHub Servo Port 4 **/

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //wristRight.setDirection(Servo.Direction.REVERSE);
        wristLeft.setDirection(Servo.Direction.REVERSE);
        //clawRight.setDirection(Servo.Direction.REVERSE);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //wristLeft.scaleRange(0,2);
        //wristRight.scaleRange(0,2);
        //clawLeft.scaleRange(0,0.1);
        //clawRight.scaleRange(0,0.1);
        //clawLeft.resetDeviceConfigurationForOpMode();
        //clawRight.resetDeviceConfigurationForOpMode();

        wristPos = 0;
        clawLeftToggle = false;
        clawRightToggle = false;

        // Retrieve the IMU from the hardware map
        //imu = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        //));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();
    }
    private void Speed() {
        //if (gamepad1.left_trigger > 0) {speed+=0.01;}
        if (gamepad1.left_trigger > 0 ) { speed1 = (gamepad1.left_trigger * maxSpeedRange1);} else if (currentGamepad1.left_trigger <=0 && previousGamepad1.left_trigger > 0) {speed1 = speed1Default;} else {speed1=speed1Default;}if (gamepad1.right_trigger > 0 /*&& maxSpeedRange1 != -1*/) {if(currentGamepad1.right_trigger != previousGamepad1.right_trigger){timeRightTriggerHeld1.reset();} if(timeRightTriggerHeld1.seconds() >= timeHeldToConfirm){maxSpeedRange1=gamepad1.right_trigger;gamepad1.rumble(50);gamepad1.setLedColor(0,1,0,-1);}else{gamepad1.setLedColor(0,0,1,-1);}} if (Math.abs(gamepad1.right_stick_x-0.3) <= 0.04 && Math.abs(gamepad1.left_trigger - 0.4) <= 0.05) {class Thread1 extends Thread{public Thread1(){this.setName("Thread1");} public void run(){while(!isInterrupted()&&opModeIsActive()){gamepad1.rumble(-1);}}} Thread thread1 = new Thread1(); /*maxSpeedRange1 = -1;*/thread1.start();}if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0 /*&& maxSpeedRange1 != -1*/) {iterationsPressed1+=1;} else {iterationsPressed1=0;} /*if (maxSpeedRange1 == -1) {speed1 = 0.2;}*/if (iterationsPressed1 >= 10) {speed1=speed1Default; maxSpeedRange1 = 1.0; iterationsPressed1=0; gamepad1.rumble(100);} if (gamepad1.right_bumper /*&& maxSpeedRange1 != -1*/) {speed1=0.25;} if (gamepad1.left_bumper /*&& maxSpeedRange1 != -1*/) {if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){previousSpeed1 = speed1; gamepad1.runRumbleEffect(maxSpeedStartUpRumbleEffect); gamepad1.runLedEffect(maxSpeedStartUpLEDEffect);} speed1 = 1; gamepad1.setLedColor(1,0,0,-1);} else if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {speed1=previousSpeed1;} else {gamepad1.setLedColor(0,0,1,-1);}
        if (gamepad2.left_trigger > 0 ) { speed2 = (gamepad2.left_trigger * maxSpeedRange2);} else if (currentGamepad2.left_trigger <=0 && previousGamepad2.left_trigger > 0) {speed2 = speed2Default;} else {speed2=speed2Default;}if (gamepad2.right_trigger > 0 /*&& maxSpeedRange2 != -1*/) {if(currentGamepad2.right_trigger != previousGamepad2.right_trigger){timeRightTriggerHeld2.reset();} if(timeRightTriggerHeld2.seconds() >= timeHeldToConfirm){maxSpeedRange2=gamepad2.right_trigger;gamepad2.rumble(50);gamepad2.setLedColor(0,1,0,-1);}else{gamepad2.setLedColor(0,0,1,-1);}} if (Math.abs(gamepad2.right_stick_x-0.3) <= 0.04 && Math.abs(gamepad2.left_trigger - 0.4) <= 0.05) {class Thread1 extends Thread{public Thread1(){this.setName("Thread1");} public void run(){while(!isInterrupted()&&opModeIsActive()){gamepad2.rumble(-1);}}} Thread thread1 = new Thread1(); /*maxSpeedRange2 = -1;*/thread1.start();}if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 /*&& maxSpeedRange2 != -1*/) {iterationsPressed2+=1;} else {iterationsPressed2=0;} /*if (maxSpeedRange2 == -1) {speed2 = 0.2;}*/if (iterationsPressed2 >= 10) {speed2=speed2Default; maxSpeedRange2 = 1.0; iterationsPressed2=0; gamepad2.rumble(100);} /*if (gamepad2.right_bumper && maxSpeedRange2 != -1) {speed2=0.25;}*/ /*if (gamepad2.left_bumper && maxSpeedRange2 != -1) {if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){previousSpeed2 = speed2; gamepad2.runRumbleEffect(maxSpeedStartUpRumbleEffect); gamepad2.runLedEffect(maxSpeedStartUpLEDEffect);} speed2 = 1; gamepad2.setLedColor(1,0,0,-1);} else if (!currentGamepad2.left_bumper && previousGamepad2.left_bumper) {speed2=previousSpeed2;} else {gamepad2.setLedColor(0,0,1,-1);}*/
        speed1 = Range.clip(speed1,0,1);
        speed2 = Range.clip(speed2,0,1);
    }
    private void fieldOriented() {
        /*double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;*/
        double yField = 0;
        if (gamepad1.dpad_up) { yField = (1);}
        if (gamepad1.dpad_down) { yField = (-1);}
        double xField = 0;
        if (gamepad1.dpad_right) { xField = (1);}
        if (gamepad1.dpad_left) { xField = (-1);}
        double rxField = gamepad1.right_stick_x;
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        /*if (gamepad1.options) {
            imu.resetYaw();
        }*/

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = xField * Math.cos(-botHeading) - yField * Math.sin(-botHeading);
        double rotY = xField * Math.sin(-botHeading) + yField * Math.cos(-botHeading);

        rotX = rotX * 1.2;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rxField), 1.0);
        double frontLeftPower = speed1 * (rotY + rotX + rxField) / denominator;
        double backLeftPower = speed1 * (rotY - rotX + rxField) / denominator;
        double frontRightPower = speed1 * (rotY - rotX - rxField) / denominator;
        double backRightPower = speed1 * (rotY + rotX - rxField) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
    private void robotOriented() {
        double yRobot = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double xRobot = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rxRobot = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(yRobot) + Math.abs(xRobot) + Math.abs(rxRobot), 1);
        double frontLeftPower = speed1 * (yRobot + xRobot + rxRobot) / denominator;
        double backLeftPower = speed1 * (yRobot - xRobot + rxRobot) / denominator;
        double frontRightPower = speed1 * (yRobot - xRobot - rxRobot) / denominator;
        double backRightPower = speed1 * (yRobot + xRobot - rxRobot) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
    private void Intake() {
        if (gamepad2.cross) {intake.setPower(1);} else if (gamepad2.circle) {intake.setPower(-1);} else {intake.setPower(0);}
    }
    private void Outtake() {
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {clawLeftToggle = !clawLeftToggle;}
        if (clawLeftToggle) {clawLeft.setPosition(maxClawPos);} else {clawLeft.setPosition(minClawPos);}
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {clawRightToggle = !clawRightToggle;}
        if (clawRightToggle) {clawRight.setPosition(maxClawPos);} else {clawRight.setPosition(minClawPos);}
        if (gamepad2.left_stick_y > 0) {wristPos -= 0.01*speed2;}
        if (gamepad2.left_stick_y < 0) {wristPos += 0.01*speed2;}
        wristPos = Range.clip(wristPos,minWristPos,maxWristPos);
        wristLeft.setPosition(wristPos);
        wristRight.setPosition(wristPos);
        telemetry.addData("wristPos: ","Left: %f, Right: %f",wristLeft.getPosition(),wristRight.getPosition());
        telemetry.addData("wristPosition: ", wristPos);
    }
    private void LiftWorks() {
        if (gamepad2.cross) {liftPosition += 55*speed2;}
        if (gamepad2.circle) {liftPosition-= 55*speed2;}
        liftPosition = Range.clip(liftPosition,0,liftMaxMotorCounts);
        liftLeft.setTargetPosition(liftPosition);
        liftRight.setTargetPosition(liftPosition);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(liftLeft.getCurrentPosition() - liftPosition) <= 20 && Math.abs(liftRight.getCurrentPosition() - liftPosition) <= 20) {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        } else {
            liftLeft.setPower(liftSpeed*speed2);
            liftRight.setPower(liftSpeed*speed2);
        }
        telemetry.addData("liftPos: ","Left: %d, Right: %d",liftLeft.getCurrentPosition(),liftRight.getCurrentPosition());
        telemetry.addData("liftPosition: ",liftPosition);
    }
    private void LiftHold() {
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(liftLeft.getCurrentPosition() - liftPosition) <= 10 && Math.abs(liftRight.getCurrentPosition() - liftPosition) <= 10) {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        } else {
            liftLeft.setPower(1);
            liftRight.setPower(1);
        }
    }
}
