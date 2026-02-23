package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Transfer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Intake, transfer and gate system")
public class Intake_Gate_Test extends OpMode {

    System_init system = new System_init();
    double gatePosL;
    double gatePosR;

    @Override
    public void init(){
        system.init(hardwareMap);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){

        if (gamepad1.left_bumper){
            system.intake.setPower(1);
        } else if (gamepad1.right_bumper){
            system.transfer.setPower(1);
        } else {
            system.intake.setPower(0);
            system.transfer.setPower(0);
        }

        if (gamepad1.squareWasPressed()){
            gatePosL += 0.1;
        } else if (gamepad1.triangleWasPressed()){
            gatePosL -= 0.1;
        } else if (gamepad1.circleWasPressed()) {
            gatePosR += 0.1;
        } else if (gamepad1.crossWasPressed()) {
            gatePosR -= 0.1;
        }

        telemetry.addData("Intake Power", system.intake.getPower());
        telemetry.addData("Transfer Power", system.transfer.getPower());
        telemetry.update();
    }

    // TODO: Get values for open gate
    public void gateOpen(){
    }

    //TODO: Get values for closed gate
    public void intake(){
        system.intake.setPower(1);
        system.transfer.setPower(1);
    }
}
