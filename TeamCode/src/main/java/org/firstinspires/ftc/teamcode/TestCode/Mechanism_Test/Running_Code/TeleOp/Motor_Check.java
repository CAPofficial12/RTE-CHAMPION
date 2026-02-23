package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

//TODO: Check if all motors are rotating ing the correct direction
@TeleOp(name = "Motor Check")
public class Motor_Check extends OpMode {
    System_init system = new System_init();
    TelemetryManager panelsTelemetry;
    
    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
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
            system.shooter.setPower(1);
        } else if (gamepad1.left_bumper) {
            system.HoodTop.setPower(1);
        } else if (gamepad1.right_bumper) {
            system.HoodTop.setPower(-1);
        } else if (gamepad1.dpad_right) {
            system.HoodTop.setTargetPosition(0);
            system.HoodTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            system.HoodTop.setPower(1);
        } else {
            system.frontLeftDrive.setPower(0);
            system.frontRightDrive.setPower(0);
            system.backRightDrive.setPower(0);
            system.backLeftDrive.setPower(0);
            system.intake.setPower(0);
            system.transfer.setPower(0);
            system.shooter.setPower(0);
            system.HoodTop.setPower(0);
        }

        telemetry.addData("Front Left", system.frontLeftDrive.getPower());
        telemetry.addData("Front Right", system.frontRightDrive.getPower());
        telemetry.addData("Back Right", system.backRightDrive.getPower());
        telemetry.addData("Back Left", system.backLeftDrive.getPower());
        telemetry.addData("Intake", system.intake.getPower());
        telemetry.addData("Transfer", system.transfer.getPower());
        telemetry.addData("Hood position", system.HoodTop.getCurrentPosition());
        telemetry.addData("Distance", system.Brake.getDistance(DistanceUnit.MM));

        telemetry.addLine("Shooter");
        panelsTelemetry.debug(system.shooter.getVelocity());
        telemetry.update();
    }
}
