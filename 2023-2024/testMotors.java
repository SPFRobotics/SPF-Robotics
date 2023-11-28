package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class testMotors extends LinearOpMode{
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor liftLeft;
    private DcMotor liftRight;
    private DcMotor intake;
    private IMU imu;

    private boolean toggleMotors = false;

    @Override
    public void runOpMode() throws InterruptedException{
        Initialization();

        while (opModeIsActive()) {
            if (gamepad1.share) { toggleMotors = !toggleMotors;}

            if (gamepad1.triangle && !toggleMotors) {frontLeft.setPower(1);} else {frontLeft.setPower(0);}
            if (gamepad1.circle && !toggleMotors) {frontRight.setPower(1);} else {frontRight.setPower(0);}
            if (gamepad1.cross && !toggleMotors) {backLeft.setPower(1);} else {backLeft.setPower(0);}
            if (gamepad1.square && !toggleMotors) {backRight.setPower(1);} else {backRight.setPower(0);}

            if (gamepad1.triangle && toggleMotors) {liftLeft.setTargetPosition(2000);liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);liftLeft.setPower(0.5);} else {liftRight.setPower(0);}
            if (gamepad1.circle && toggleMotors) {liftRight.setTargetPosition(2000);liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);liftRight.setPower(0.5);} else {liftRight.setPower(0);}
            if (gamepad1.cross && toggleMotors) {intake.setPower(1);} else {intake.setPower(0);}
            //if (gamepad1.square && toggleMotors) {frontLeft.setPower(1);} else {frontLeft.setPower(0);}
        }
    }

    private void Initialization(){
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeft = hardwareMap.dcMotor.get("frontLeft"); /** Port: ControlHub MotorPort 0 **/
        backLeft = hardwareMap.dcMotor.get("backLeft"); /** Port: ControlHub MotorPort 2 **/
        frontRight = hardwareMap.dcMotor.get("frontRight"); /** Port: ControlHub MotorPort 1 **/
        backRight = hardwareMap.dcMotor.get("backRight"); /** Port: ControlHub MotorPort 3 **/
        liftLeft = hardwareMap.dcMotor.get("liftLeft"); /** Port: ExpansionHub MotorPort 3 **/
        liftRight = hardwareMap.dcMotor.get("liftRight"); /** Port: ExpansionHub MotorPort 2 **/
        intake = hardwareMap.dcMotor.get("intake");  /** Port: ExpansionHub MotorPort 1 **/

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
}
