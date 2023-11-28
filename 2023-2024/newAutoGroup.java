package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class newAutoGroup extends LinearOpMode {
    MecanumChassis mc = new MecanumChassis(this);
    Intake it = new Intake(this);

    public void runOpMode(){
        mc.initializeMovement();
        mc.move(.4, "left", 23);
    }
}
