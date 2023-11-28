package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous


public class testEncoderMotors extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private IMU imu;
    @Override
    public void runOpMode() {
        Initializtion();

        if (gamepad1.triangle) {frontLeft.setTargetPosition((int)frontLeft.getCurrentPosition()+100);}
        if (gamepad1.circle) {frontRight.setTargetPosition((int)frontRight.getCurrentPosition()+100);}
        if (gamepad1.cross) {backLeft.setTargetPosition((int)backLeft.getCurrentPosition()+100);}
        if (gamepad1.square) {backRight.setTargetPosition((int)backRight.getCurrentPosition()+100);}
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(0.25);
        backRight.setPower(0.25);
        frontLeft.setPower(0.25);
        frontRight.setPower(0.25);
    }

    private void Initializtion() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); /** Port: ControlHub MotorPort 0 **/
        backLeft = hardwareMap.dcMotor.get("backLeft"); /** Port: ControlHub MotorPort 2 **/
        frontRight = hardwareMap.dcMotor.get("frontRight"); /** Port: ControlHub MotorPort 1 **/
        backRight = hardwareMap.dcMotor.get("backRight"); /** Port: ControlHub MotorPort 3 **/
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
    }

}
