package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Transfer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake, transfer and gate system")
public class Intake_Gate extends OpMode {

    DcMotorSimple Intake;
    Servo gate;
    double gateClose;
    double gateOpen;
    boolean Config;

    @Override
    public void init(){
        Intake = hardwareMap.get(DcMotorSimple.class, "Intake");
        gate = hardwareMap.get(Servo.class, "Gate");
        gateClose = gate.getPosition();
        gateOpen = gateClose;

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop(){

        if (Config) {
            if (gamepad1.square) {
                Intake.setPower(1);
            } else if (gamepad1.triangle) {
                Intake.setPower(-1);
            } else if (gamepad1.circle) {
                Intake.setPower(0);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                gateOpen += 0.1;
            } else if (gamepad1.dpadRightWasPressed()) {
                gateOpen -= 0.1;
            }

            if (gamepad1.dpad_up){
                gate.setPosition(gateOpen);
            } else{
                gate.setPosition(gateClose);
            }

            telemetry.addData("Servo Pos", gate.getPosition());
            telemetry.addData("Intake Power", Intake.getPower());
            telemetry.update();

        } else {

            if (gamepad1.square) {
                Intake.setPower(1);
            } else if (gamepad1.circle) {
                Intake.setPower(0);
            }

            if (gamepad1.right_bumper){     //TODO : create method way to check if a part of the robot intersects the launch zones
                gate.setPosition(gateOpen);
            } else{
                gate.setPosition(gateClose);
            }

        }

        if (gamepad1.leftStickButtonWasPressed()){
            Config = true;
        } else if(gamepad1.rightStickButtonWasReleased()){
            Config = false;
        }
    }
}
