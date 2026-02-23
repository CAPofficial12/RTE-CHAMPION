package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Transfer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Intake Test")
public class Intaker_transfer_tets extends OpMode {
    System_init system = new System_init();
    double distance;

    @Override
    public void init(){
        system.init(hardwareMap);
    }

    @Override
    public void loop(){
        distance = system.Brake.getDistance(DistanceUnit.MM);

        if (gamepad1.dpad_up && distance > 90 || gamepad1.right_trigger > 0.05) {
            system.intake.setPower(1);
            system.transfer.setPower(1);
        } else if (gamepad1.dpad_up && distance < 90) {
            system.intake.setPower(1);
            system.transfer.setPower(0);
        } else if (gamepad1.dpad_down) {
            system.intake.setPower(-1);
        } else if (gamepad1.cross) {
            system.intake.setPower(-1);
            system.transfer.setPower(-1);

        } else {
            system.transfer.setPower(0);
            system.intake.setPower(0);
        }

        if (gamepad1.right_bumper){
            system.shooter.setVelocity(2000);
        } else {
            system.shooter.setVelocity(0);
        }

        telemetry.addData("Open/ Close", distance);
        telemetry.addData("shootern speed", system.shooter.getVelocity());
        telemetry.update();

    }
}
