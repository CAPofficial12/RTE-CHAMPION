package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Shooter Tuning")
public class Shooter_tune extends OpMode {
    System_init system_init = new System_init();

    double[] step = {10000, 1000,100,10,1,0.1,0.01,0.001, 0.001};

    int a = 0;
    int mode = 1;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    double target_speed = 1000;
    double testing_hood = 0;

    @Override
    public void init() {
        system_init.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        testing_hood = system_init.Hood.getPosition();
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.rightStickButtonWasReleased()){
            target_speed += mode * step[a];
        }

        if (gamepad1.rightBumperWasPressed()) {
            a += 1;
        } else if (gamepad1.leftBumperWasPressed()) {
            a -= 1;
        }

        if (gamepad1.dpad_right) {
            mode = 1;
        } else if (gamepad1.dpad_left) {
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
        } else if (gamepad1.dpadUpWasPressed()){
            testing_hood += step[a];
        } else if (gamepad1.dpadLeftWasPressed()) {
            testing_hood += step[a];
        } else if (gamepad1.dpadLeftWasPressed()) {
            target_speed = 1000;
        } else if (gamepad1.dpadRightWasPressed()){
            target_speed = 2000;
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        system_init.shooter.setVelocity(target_speed);
        system_init.Hood.setPosition(testing_hood);
        double currentVelocity = system_init.shooter.getVelocity();
        double error = target_speed - currentVelocity;

        telemetry.addData("Target Velocity", target_speed);
        telemetry.addData("Current velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Mode", mode);
        telemetry.addData("Step", step[a]);
        telemetry.addLine("------------------------------------------");
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("Kf", Kf);
        telemetry.update();
    }
    public double[] StatCalc(){ //TODO: Get an approximation of an equation for the shooter velocity and hood angle needed to score from each distance
        double[] palceholder = {1,2};
        return palceholder;
    }
}


