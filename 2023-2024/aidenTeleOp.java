package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class aidenTeleOp extends LinearOpMode {
    DcMotor frontLeft = null;
    DcMotor backLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;
    IMU imu = null;
    DcMotor liftLeft = null;
    DcMotor liftRight = null;
    DcMotor intake = null;
    Servo wristLeft = null;
    Servo wristRight = null;
    Servo clawLeft = null;
    Servo clawRight = null;

    public void initialize(){
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft"); /** Port: ExpansionHub MotorPort 3 **/
        liftRight = hardwareMap.dcMotor.get("liftRight"); /** Port: ExpansionHub MotorPort 2 **/
        intake = hardwareMap.dcMotor.get("intake");  /** Port: ExpansionHub MotorPort 1 **/
        wristLeft = hardwareMap.servo.get("wristLeft"); /** Port: ExpansionHub ServoPort 4 **/
        wristRight = hardwareMap.servo.get("wristRight"); /** Port: ExpansionHub ServoPort 5 **/
        clawLeft = hardwareMap.servo.get("clawLeft"); /** Port: ControlHub Servo Port 5 **/
        clawRight = hardwareMap.servo.get("clawRight"); /** Port: ControlHub Servo Port 4 **/

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        wristLeft.setDirection(Servo.Direction.REVERSE);
        clawLeft.setDirection(Servo.Direction.REVERSE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wristLeft.scaleRange(0,2);
        wristRight.scaleRange(0,2);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot | ORIENTATION IS GUESSWORK DOUBLE CHECK!!!!!!
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void fieldCentric(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
            /*if (gamepad1.options) {
                imu.resetYaw();
            }*/

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing | SHOULD BE 1.2??????

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void intake(){
        if (gamepad2.cross) {
            intake.setPower(1);
        } else if (gamepad2.circle) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            fieldCentric(); //Checks inputs for movement
            intake(); //Checks inputs for intake system
        }
    }
}
