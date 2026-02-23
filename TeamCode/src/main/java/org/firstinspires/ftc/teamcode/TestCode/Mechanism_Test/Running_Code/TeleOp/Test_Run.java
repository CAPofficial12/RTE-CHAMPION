package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp (name = "Robot go brrr")
public class Test_Run extends OpMode {
    System_init system = new System_init();

    @Override
    public void init(){
        system.init(hardwareMap);
    }

    @Override
    public void loop(){

        if (gamepad1.square){
            system.intake.setPower(1);
        } else{
            system.intake.setPower(0);
        }

        if (gamepad1.triangle) {
            system.transfer.setPower(1);
        } else {
            system.transfer.setPower(0);
        }

    }
}
