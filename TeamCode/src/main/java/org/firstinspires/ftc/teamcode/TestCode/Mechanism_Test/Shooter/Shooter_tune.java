package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Shooter Tuning")
public class Shooter_tune extends OpMode {
    System_init system_init = new System_init();
    TelemetryManager telemetryM;
    PIDFCoefficients pidfCoefficients;

    double[] step = {10000, 1000,100,10,1,0.1,0.01,0.001, 0.001};
    int a = 0;
    int mode = 1;
    double Kp = Shooter_Constants.Kp;
    double Ki = Shooter_Constants.Ki;
    double Kd = Shooter_Constants.Kd;
    double Kf = Shooter_Constants.Kf;
    double target_speed = Shooter_Constants.target_speed;
    int testing_hoodT = Shooter_Constants.target_hood;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        system_init.init(hardwareMap);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        testing_hoodT = system_init.HoodTop.getCurrentPosition();
        system_init.HoodTop.setTargetPosition(testing_hoodT);
        system_init.HoodTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        system_init.HoodTop.setPower(1);

        telemetryM.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.rightStickButtonWasReleased()){
            target_speed += step[a];
        } else if (gamepad1.leftStickButtonWasPressed()){
            target_speed -= step[a];
        }

        if (gamepad1.rightBumperWasPressed()) {
            a += 1;
        } else if (gamepad1.leftBumperWasPressed()) {
            a -= 1;
        }

        if (gamepad1.dpadRightWasPressed()){
            testing_hoodT += (int) step[a];
        } else if (gamepad1.dpadLeftWasPressed()) {
            testing_hoodT -= (int) step[a];
        }

        if (gamepad1.dpad_up){
            mode = 1;
        } else if (gamepad1.dpad_down) {
            mode = -1;
        }

        if (gamepad1.squareWasPressed()) {
            Kp += mode * step[a];
        } else if (gamepad1.triangleWasPressed()) {
            Ki += mode * step[a];
        } else if (gamepad1.circleWasPressed()) {
            Kd += mode * step[a];
        } else if (gamepad1.crossWasPressed()) {
            Kf += mode * step[a];
        }
        pidfCoefficients = new PIDFCoefficients(Kp, Ki,Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        system_init.HoodTop.setTargetPosition(testing_hoodT);
        system_init.shooter.setVelocity(target_speed);

        double currentVelocity = system_init.shooter.getVelocity();
        double error = target_speed - currentVelocity;

        system_init.intake.setPower(gamepad1.right_trigger);
        system_init.transfer.setPower(gamepad1.right_trigger);

        telemetryM.addData("Target Velocity", target_speed);
        telemetryM.addData("Current velocity", currentVelocity);
        telemetryM.addData("Error", error);
        telemetryM.addData("Mode", mode);
        telemetryM.addData("Step", step[a]);
        telemetryM.addData("Hood angle", testing_hoodT);
        telemetryM.addData("True Hood Angle", system_init.HoodTop.getCurrentPosition());
        telemetryM.addLine("------------------------------------------");
        telemetryM.addData("Kp", Kp);
        telemetryM.addData("Ki", Ki);
        telemetryM.addData("Kd", Kd);
        telemetryM.addData("Kf", Kf);
        telemetryM.update(telemetry);
    }

    //TODO: Get an approximation of an equation for the shooter velocity and hood angle needed to score from each distance
    public double[] StatCalc(){
        double[] placeholder = {1,2};
        return placeholder;
    }
}