package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public LinearOpMode opmode = null;
    public Servo wristServoLeft = null;
    public Servo wristServoRight = null;
    public double minWristPos = 0;
    public double maxWristPos = .9;

    public Wrist(LinearOpMode lom){
        opmode = lom;
        wristServoLeft = opmode.hardwareMap.get(Servo.class, "wristLeft");
        wristServoRight = opmode.hardwareMap.get(Servo.class, "wristRight");
    }
    public void goToStart(){
        wristServoLeft.setPosition(minWristPos);
        wristServoLeft.setPosition(minWristPos);
    }
    public void goToMax(){
        wristServoLeft.setPosition(maxWristPos);
        wristServoLeft.setPosition(maxWristPos);
    }
}