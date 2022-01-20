package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="[TEST] Lift")
public class Lift extends LinearOpMode {

    // define hardware devices
    private DcMotor lift1;
    private DcMotor lift2;

    //define variables
    double multiplier;
    double dustbinPos = .35;

    @Override
    public void runOpMode() {

        lift1 = hardwareMap.get(DcMotor.class, "linearSlideBottom");
        lift2 = hardwareMap.get(DcMotor.class, "linearSlideTop");


        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                {
                    // gamepad2.left_stick_y moves the lift up or down.
                    // only move one motor at a time.
                    // move lift1 up first.
                    // if lift1.getCurrentPosition() reaches it's max value (9900), switch to moving lift2.
                    // if lift2.getCurrentPosition() reaches it's max value (5000), do not allow the lift to move further up, but allow moving further down.
                    // if lift2.getCurrentPosition() reaches it's min value (0), switch to moving lift1
                    // if lift1.getCurrentPosition() reaches it's min value (0), do not allow the lift to move further down, but allow moving further up.

                    if (gamepad2.left_stick_y > 0) {
                        lift1.setPower(gamepad2.left_stick_y);
                        lift2.setPower(0);
                    } else if (gamepad2.left_stick_y < 0) {
                        lift1.setPower(0);
                        lift2.setPower(gamepad2.left_stick_y);
                    } else {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }

                    if (gamepad2.left_stick_y == 0) {
                        if (lift1.getCurrentPosition() == 9900) {
                            lift1.setPower(0);
                            lift2.setPower(gamepad2.left_stick_y);
                        } else if (lift2.getCurrentPosition() == 5000) {
                            lift1.setPower(gamepad2.left_stick_y);
                            lift2.setPower(0);
                        } else if (lift2.getCurrentPosition() == 0) {
                            lift1.setPower(0);
                            lift2.setPower(gamepad2.left_stick_y);
                        } else if (lift1.getCurrentPosition() == 0) {
                            lift1.setPower(gamepad2.left_stick_y);
                            lift2.setPower(0);
                        }
                    }

                    if(gamepad2.left_stick_y==0) {
                        lift1.setPower(0);
                        lift2.setPower(0);
                    }
                }

                telemetry.update();
            }
        }
    }
}
