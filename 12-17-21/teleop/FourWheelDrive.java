package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FourWheelDrive")
public class FourWheelDrive extends LinearOpMode {

    // define hardware devices
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor lift;
    private DcMotor carousel;
    private CRServo collector;
    private Servo leftServo;
    private Servo rightServo;

    //define variables
    double multiplier;
    double dustbinPos = .35;

    public void runOpMode() {

        //configure hardware devices
        System.out.println(hardwareMap.allDeviceMappings);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        carousel = hardwareMap.get(DcMotor.class,"carousel");
        collector = hardwareMap.get(CRServo.class, "collector");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        {   // calculate speed multiplier

            if (gamepad1.b) multiplier=1;
            else if (gamepad1.y) multiplier=.25;
            else if (gamepad1.left_bumper) multiplier=1;
            else if (gamepad1.right_bumper) multiplier=.25;
            else multiplier=.5;

        }

        {   // Move Mecanum Wheels with gamepad1 sticks

            frontLeft.setPower(multiplier * (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            backLeft.setPower(multiplier * (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            frontRight.setPower(multiplier * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            backRight.setPower(multiplier * (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));

        }

        {   // move lift with gamepad2 left stick

            if (lift.getCurrentPosition() < -2225) lift.setPower(-0.15);
            else if (lift.getCurrentPosition() > 9500) lift.setPower(0.15);
            else lift.setPower(gamepad2.left_stick_y);
        }

        {   // move dustbin servos opposite of each other with gamepad2 right stick

            if(dustbinPos<0) dustbinPos = 0;
            else if(dustbinPos>1) dustbinPos = 1;

            rightServo.setPosition(dustbinPos);
            leftServo.setPosition(1-dustbinPos);

        }

        {   // spin collector with gamepad2 dpad

            if (gamepad2.dpad_up) collector.setPower(1);
            else if (gamepad2.dpad_down) collector.setPower(-1);
            else collector.setPower(0);

        }

        {   // spin carousel with gamepad2 triggers

            if (gamepad2.b) carousel.setPower((gamepad2.right_trigger - gamepad2.left_trigger) / 3);
            else carousel.setPower((gamepad2.right_trigger - gamepad2.left_trigger) / 5.5);

        }
    }
}
