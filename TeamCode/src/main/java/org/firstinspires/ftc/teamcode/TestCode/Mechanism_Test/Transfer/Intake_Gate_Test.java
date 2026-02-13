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
        gatePosL = system.gateLeft.getPosition();
        gatePosR = system.gateRight.getPosition();
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

        system.gateRight.setPosition(gatePosL);
        system.gateLeft.setPosition(gatePosR);

        telemetry.addData("Intake Power", system.intake.getPower());
        telemetry.addData("Transfer Power", system.transfer.getPower());
        telemetry.addData("Gate Left position", system.gateLeft.getPosition());
        telemetry.addData("Gate Right position", system.gateRight.getPosition());
        telemetry.update();
    }

    public void gateOpen(){
        system.gateLeft.setPosition(0);
        system.gateRight.setPosition(0);
    }

    public void gateClose(){
        system.gateLeft.setPosition(1);
        system.gateRight.setPosition(1);
    }

    public void intake(){
        system.intake.setPower(1);
        system.transfer.setPower(1);
    }
}
