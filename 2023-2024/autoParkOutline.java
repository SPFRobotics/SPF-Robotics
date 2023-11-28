package org.firstinspires.ftc.teamcode;

//These are imports for all of the libraries you need to run the program
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import java.util.Arrays;
//These imports are should be in most if not all programs
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//For checking time
import com.qualcomm.robotcore.util.ElapsedTime;
//IMU imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//"@Autonomous" puts this program in the autonomous dropdown on the driverhub
@Autonomous
public class autoParkOutline extends LinearOpMode { //"extends LinearOpMode" just means that this program can use motors, also only one of the programs running is allowed to have the "LinearOpMode"
    //"@Override" should be before your runOpMode function
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private  IMU imu;
    @Override
    public void runOpMode(){
        Initializtion();

        //moving forward 5 inches
        backLeft.setTargetPosition((int) inch_convert(5));
        backRight.setTargetPosition((int) inch_convert(5));
        frontLeft.setTargetPosition((int) inch_convert(5));
        frontRight.setTargetPosition((int) inch_convert(5));
        run_to_position_all();
        backLeft.setPower(0.25);
        backRight.setPower(0.25);
        frontLeft.setPower(0.25);
        frontRight.setPower(0.25);
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("test", "attempting to move...");
            telemetry.update();
        }
        powerZero();
    }
    private void Initializtion() {
        //making variables for each DcMotor and defining each variable to their corresponding motor, word in quotes is the name of the motor in the configuration of the robot
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");

        //this sets all of the motor's zero power behavior (when the motor is not moving) to brake
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        stop_and_reset_encoders_all();
        
        //instializing the IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw(); //setting yaw to zero incase it isn't already
        
        //everything before this will run when you click the "int" button on the driver hub, and when the following line is just waiting until you click start button
        waitForStart();
    }
    
    //3.78 is the diameter of the wheel, and 537.7 is how many motor counts are in 1 full rotation of the motor's axle
    private double inch_convert(double inch) { return inch * (537.7 / (3.78 * Math.PI)); }
    private double inToCm(int inches) { return inches * 2.54; }
    private double cm_convert(double cm) { return cm * (537.7 / (9.6012 / Math.PI)); }
    
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
}
