package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    DcMotorEx Intake;

    public void Inhale(){
        Intake.setPower(1);
    }

    public void Exhale(){
        Intake.setPower(-1);
    }
}
