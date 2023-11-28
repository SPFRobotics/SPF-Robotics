package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class autoTest1_EK extends LinearOpMode{
    //public static boolean RunAutoRight = false;
    //public static boolean RunMoveToCone = false;
    //ElapsedTime AutoRightTime = new ElapsedTime();
    //ElapsedTime MoveToConeTime = new ElapsedTime();
    private static final double strafeMult = 1.2;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;
    private IMU imu = null;
    private int[] xCords = new int[] {0,1,2}; //right to left (looking from alliance station)
    private int[] yCords = new int[] {0,1,2}; //1:A, 2:B, 3:C, 4:D, 5:E, 6:F | front to back (front being closest row to alliance station)
    private int[] startCords = new int[] {xCords[1],yCords[0]}; //starting locations; Blue: A2(0,1),A5(0,4); Red: F2(5,1),F5(5,4)
    //don't change these values, change the values in the PointMove function instead
    private int[] endCords = new int[] {xCords[0],yCords[0]}; //change the xCords and yCords value to change defalt end location

    @Override
    public void runOpMode() throws InterruptedException {
        //By Friday, have it be able to
        Initializtion();
        rotate(90,0.3);
        //move(.3, "backward", 12);
        //move(.3, "left", 10);
        //move(.3, "right", 8);
        //move(.3, "forward", 6);
    }

    //3.78(in inches, 9.6012 is centimeters) is the diameter of the wheel, and 537.7 is how many motor counts are in 1 full rotation of the motor's axle
    private double inch_convert(double inch) { return inch * (537.7 / (3.78 * Math.PI)); }
    private double inToCm(int inches) { return inches * 2.54; }
    private double cm_convert(double cm) { return cm * (537.7 / (9.6012 / Math.PI)); }

    private void Initializtion() {
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
        waitForStart();
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
        if (moveDirection.equals("backward")) {
            backLeft.setTargetPosition((int) inch_convert(moveDistance));
            backRight.setTargetPosition((int) inch_convert(moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance));
            frontRight.setTargetPosition((int) inch_convert(moveDistance));
            run_to_position_all();
            backLeft.setPower(movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(movePower);
        } else if (moveDirection.equals("forward")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance));
            backRight.setTargetPosition((int) inch_convert(-moveDistance));
            frontLeft.setTargetPosition((int) inch_convert(-moveDistance));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(-movePower);
            frontLeft.setPower(-movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("left")) {
            backLeft.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            backRight.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontLeft.setTargetPosition((int) inch_convert(moveDistance*strafeMult));
            frontRight.setTargetPosition((int) inch_convert(-moveDistance*strafeMult));
            run_to_position_all();
            backLeft.setPower(-movePower);
            backRight.setPower(movePower);
            frontLeft.setPower(movePower);
            frontRight.setPower(-movePower);
        } else if (moveDirection.equals("right")) {
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
    private void PointMove(int endPosX, int endPosY) {
        int[] pointDifference = new int[] {0,0};
        endCords[0] = xCords[endPosX];
        endCords[1] = yCords[endPosY];

        stop_and_reset_encoders_all();
        waitForStart();
        if (opModeIsActive()) {
            pointDifference[0] = endCords[0] - startCords[0];
            pointDifference[1] = endCords[1] - startCords[1];
            move(0.25, "left", pointDifference[0]*24);
            move(0.25, "forward", pointDifference[1]*24);
            startCords[0] = endCords[0];
            startCords[1] = endCords[1];
        }
    }
}