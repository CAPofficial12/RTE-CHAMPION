package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Motor Check")
public class Motor_Check extends OpMode {
    System_init system = new System_init();
    
    @Override
    public void init(){
        system.init(hardwareMap);
    }
    
    @Override
    public void loop(){
        if (gamepad1.triangle){
            system.frontLeftDrive.setPower(1);
        } else if (gamepad1.circle) {
            system.frontRightDrive.setPower(1);
        } else if (gamepad1.cross) {
            system.backRightDrive.setPower(1);
        } else if (gamepad1.square) {
            system.backLeftDrive.setPower(1);
        } else if (gamepad1.dpad_up) {
            system.intake.setPower(1);
        } else if (gamepad1.dpad_down) {
            system.transfer.setPower(1);
        } else if (gamepad1.dpad_left) {
            system.shooter.setVelocity(2000);
        } else {
            system.frontLeftDrive.setPower(0);
            system.frontRightDrive.setPower(0);
            system.backRightDrive.setPower(0);
            system.backLeftDrive.setPower(0);
            system.intake.setPower(0);
            system.transfer.setPower(0);
            system.shooter.setPower(0);
        }

        telemetry.addData("Front Left", system.frontLeftDrive.getPower());
        telemetry.addData("Front Right", system.frontRightDrive.getPower());
        telemetry.addData("Back Right", system.backRightDrive.getPower());
        telemetry.addData("Back Left", system.backLeftDrive.getPower());
        telemetry.addData("Intake", system.intake.getPower());
        telemetry.addData("Transfer", system.transfer.getPower());
        telemetry.addData("Shooter", system.shooter.getVelocity());
        telemetry.update();
    }
}
