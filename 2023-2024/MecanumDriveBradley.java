package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

//Mr. Bradley the name of the class was different from name of file and was causing errors, so I corrected it -Eli
//public class MecanumDrive {
public class MecanumDriveBradley {

    private LinearOpMode opmode = null;
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor leftFrontWheel = null;
    private DcMotor rightFrontWheel = null;
    private DcMotor leftBackWheel = null;
    private DcMotor rightBackWheel = null;

    private IMU imu = null;


    //CONSTANTS
    private static final double WHEEL_DIAMETER = 3.78; //This is in inches. Team needs to decide on units.
    private static final double TICKS_PER_ROTATION = 537.6; //This is for the GoBilda motors.
    //******* TODO: My want to add other constants like TICKS_PER_INCH?
    private double TICKS_PER_CM = ( 537.6/(10*Math.PI));

    //Enumeration type used to specify direction when sending a move command.
    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
    //also changed from file name error
    //MecanumDrive(LinearOpMode creatingOpMode){
    MecanumDriveBradley(LinearOpMode creatingOpMode){

        //need to pass the OpMode that create the object so you can initialize the hardware,
        //monitor OpModeActive and send telemetry
        opmode = creatingOpMode;
    }

    public void initialize(){

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontWheel = opmode.hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontWheel = opmode.hardwareMap.get(DcMotor.class, "frontRight");
        leftBackWheel = opmode.hardwareMap.get(DcMotor.class, "backLeft");
        rightBackWheel = opmode.hardwareMap.get(DcMotor.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);

        //Make sure motors are stopped an reset encoder.
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set Motors brake mode(Zero Behavior Mode)
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set motors run mode (could be RUN_USING_ENCODER or RUN_WITHOUT_ENCODER)
        //Might be boned here
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize IMU: The IMU provides heading/direction of the robot. It can be used to turn
        // the robot and also could be used to help keep the robot straight when moving forward or strafing.
        //****** TODO: See SensorIMUOrthoganal.java sample code lines 108-115
    }

    /**
     * This a method to drive omni-wheels. See sample code BasicOmniOpMode_Linear.
     * @param axial: power forward/backwards
     * @param lateral: power left/right (Note: positive power is left)
     * @param yaw: rotation. CCW is positive
     */
    public void drive(double axial, double lateral, double yaw){

        // Combine the power for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontWheel  = axial + lateral + yaw;
        //*****TODO: See BasicOmniOpMode_Linear example. Also check out GMO https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        //****TODO: Same reference above. See code lines 127-136 on BasicOmniOpMode_Linear

        // Send calculated power to wheels
        //****TODO: See code lines 156-159

        // Good Idea to use telemetry to display data on drive hub for debugging.
        //****TODO: See code lines 162-165
    }

    /**
     * This method implements FOV drive method described in GM 0 reference:
     * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
     * @param axial
     * @param lateral
     * @param yaw
     */
    public void driveFOV(double axial, double lateral, double yaw){

        //*****TODO: Look at sample code in GM O and implement.
    }

    /**
     * This method uses RUN_TO_POSITION mode to drive the robot a the distance specified. The OpMode
     * that calls this method will need to use the isBusy() method to check when motors are done
     * executing the move
     * @param direction
     * @param distance
     * @param power
     */
    public void driveEncoder(Direction direction, double distance, double power) {

        double distanceCm = TICKS_PER_CM * distance;

        int newleftFrontTarget = 0;
        int newrightFrontTarget = 0;
        int newrightBackTarget = 0;
        int newleftBackTarget = 0;

        switch (direction){
            case FORWARD:
                newleftFrontTarget = leftFrontWheel.getCurrentPosition()+ (int) distanceCm;
                newrightFrontTarget = rightFrontWheel.getCurrentPosition()+ (int)distanceCm;
                newrightBackTarget = rightBackWheel.getCurrentPosition()+ (int)distanceCm;
                newleftBackTarget = leftBackWheel.getCurrentPosition()+ (int)distanceCm;
                break;
            case BACKWARD:
                newleftFrontTarget = leftFrontWheel.getCurrentPosition()- (int) distanceCm;
                newrightFrontTarget = rightFrontWheel.getCurrentPosition()- (int)distanceCm;
                newrightBackTarget = rightBackWheel.getCurrentPosition()- (int)distanceCm;
                newleftBackTarget = leftBackWheel.getCurrentPosition()- (int)distanceCm;
                break;
            case LEFT:
                newleftFrontTarget = leftFrontWheel.getCurrentPosition()- (int) distanceCm;
                newrightFrontTarget = rightFrontWheel.getCurrentPosition()+ (int)distanceCm;
                newrightBackTarget = rightBackWheel.getCurrentPosition()- (int)distanceCm;
                newleftBackTarget = leftBackWheel.getCurrentPosition()+ (int)distanceCm;
                break;
            case RIGHT:
                newleftFrontTarget = leftFrontWheel.getCurrentPosition()+ (int) distanceCm;
                newrightFrontTarget = rightFrontWheel.getCurrentPosition()- (int)distanceCm;
                newrightBackTarget = rightBackWheel.getCurrentPosition()+ (int)distanceCm;
                newleftBackTarget = leftBackWheel.getCurrentPosition()- (int)distanceCm;
                break;
            //TODO: Add other movements like the diagonal ones
        }

        //Set new target positions for each motor
        leftFrontWheel.setTargetPosition(newleftFrontTarget);
        leftBackWheel.setTargetPosition(newleftBackTarget);
        rightBackWheel.setTargetPosition(newrightBackTarget);
        rightFrontWheel.setTargetPosition(newrightFrontTarget);

        //Turn on RUN_TO_POSITION
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Power
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightBackWheel.setPower(power);
        rightFrontWheel.setPower(power);

        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Method checks to see if motors are still busy executing a move
     * @return
     */
    public boolean isBusy(){

        opmode.telemetry.addData("Currently at",  " at %7d",
                leftBackWheel.getCurrentPosition());
        opmode.telemetry.update();

        return (leftFrontWheel.isBusy() || leftBackWheel.isBusy() || rightFrontWheel.isBusy() || rightBackWheel.isBusy());
    }

    /**
     * Method turns the robot to a heading.
     * @param heading
     * @param power
     */
    public void turnToHeading(double heading, double power){

        //TODO: Write code to turn robot using the IMU
    }

    /**
     * Method returns current position of encoders for each wheel. Order is LeftFront,RightFront,
     * LeftBack, RightBack
     *
     * @return
     */
    public double[] getCurrentPosition(){
        double[] currPositions = new double[4];
        //****** TODO: read current positions of encoders and return
        return(currPositions);
    }


}

