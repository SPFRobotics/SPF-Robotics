/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//need lift object, drive object, intake and outtake objects as well
public class teleOpObjects {
    public class teleOpCombinedDrive {
        //
    }
    @TeleOp
    public class teleOpCombinedDrivesOneDriver1 extends LinearOpMode {
        private DcMotor backLeft;
        private DcMotor backRight;
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor liftLeft;
        private DcMotor liftRight;
        private DcMotor intake;
        private IMU imu;
        private double speed = 0.5;
        private double liftSpeed = 1.0;
        private static final int liftMaxMotorCounts = 4062;
        private double previousSpeed1;
        private double previousSpeed2;
        private int iterationsPressed = 0;
        private Gamepad currentGamepad1;
        private Gamepad previousGamepad1;
        private ElapsedTime timeRightTriggerHeld = new ElapsedTime();
        private double timeHeldToConfirm = 0.5;
        int maxSpeedDuration = 50;
        private double maxSpeedRange = 1.0;
        private int liftPosition = 0;
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
        public void runOpMode() throws InterruptedException {
            Initialization();

            if (isStopRequested()) return;

            currentGamepad1 = new Gamepad();
            previousGamepad1 = new Gamepad();

            while (opModeIsActive()) {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                Speed();
                if (Math.abs(gamepad1.left_stick_x) <= 0.05 && Math.abs(gamepad1.left_stick_y) <= 0.05) {
                    fieldOriented();
                } else {
                    robotOriented();
                }
                if (gamepad1.cross) {liftPosition += 55*speed;}
                if (gamepad1.circle) {liftPosition-= 55*speed;}
                if (gamepad1.square) {intake.setPower(1);} else if (gamepad1.triangle) {intake.setPower(-1);} else {intake.setPower(0);}
                liftPosition = Range.clip(liftPosition,0,liftMaxMotorCounts);
                liftLeft.setTargetPosition(liftPosition);
                liftRight.setTargetPosition(liftPosition);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(liftLeft.getCurrentPosition() - liftPosition) <= 20 && Math.abs(liftRight.getCurrentPosition() - liftPosition) <= 20) {
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                } else {
                    liftLeft.setPower(liftSpeed*speed);
                    liftRight.setPower(liftSpeed*speed);
                }
                telemetry.addData("liftPos: ","Left: %d, Right: %d",liftLeft.getCurrentPosition(),liftRight.getCurrentPosition());
                telemetry.addData("liftPosition: ",liftPosition);
                telemetry.update();
            }
        }

        private void Initialization() {
            // Declare our motors
            // Make sure your ID's match your configuration
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backRight = hardwareMap.dcMotor.get("backRight");
            liftLeft = hardwareMap.dcMotor.get("liftLeft");
            liftRight = hardwareMap.dcMotor.get("liftRight");
            intake = hardwareMap.dcMotor.get("intake");

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

            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Retrieve the IMU from the hardware map
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            imu.resetYaw();
            waitForStart();
        }
        private void Speed() {
            //if (gamepad1.left_trigger > 0) {speed+=0.01;}
            speed = Range.clip(speed,0,1);
        }
        private void fieldOriented() {
        //double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        //double x = -gamepad1.left_stick_x;
        //double rx = gamepad1.right_stick_x;
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
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = xField * Math.cos(-botHeading) - yField * Math.sin(-botHeading);
            double rotY = xField * Math.sin(-botHeading) + yField * Math.cos(-botHeading);

            rotX = rotX * 1.2;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rxField), 1);
            double frontLeftPower = speed * (rotY + rotX + rxField) / denominator;
            double backLeftPower = speed * (rotY - rotX + rxField) / denominator;
            double frontRightPower = speed * (rotY - rotX - rxField) / denominator;
            double backRightPower = speed * (rotY + rotX - rxField) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
        private void robotOriented() {
            double yRobot = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double xRobot = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
            double rxRobot = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(yRobot) + Math.abs(xRobot) + Math.abs(rxRobot), 1);
            double frontLeftPower = speed * (yRobot + xRobot + rxRobot) / denominator;
            double backLeftPower = speed * (yRobot - xRobot + rxRobot) / denominator;
            double frontRightPower = speed * (yRobot - xRobot - rxRobot) / denominator;
            double backRightPower = speed * (yRobot + xRobot - rxRobot) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}*/