package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    public LinearOpMode opmode = null;
    public DcMotor intakeMotor = null;
    public Intake(LinearOpMode lom){
        opmode = lom;
        intakeMotor = opmode.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void intakeOn(double power){
        intakeMotor.setPower(power);
    }
    public void intakeOff(){
        intakeMotor.setPower(0);
    }
}
