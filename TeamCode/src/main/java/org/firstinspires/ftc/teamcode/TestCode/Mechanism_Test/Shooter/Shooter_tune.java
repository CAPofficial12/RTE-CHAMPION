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
    double testing_hoodT = 0;
    double testing_hoodR = 0;
    double testing_hoodL = 0;

    @Override
    public void init() {
        system_init.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        testing_hoodT = system_init.HoodTop.getPosition();
        testing_hoodL = system_init.HoodLeft.getPosition();
        testing_hoodR = system_init.HoodRight.getPosition();
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad1.rightStickButtonWasReleased()){
            target_speed += step[a];
        } else if (gamepad1.leftBumperWasPressed()){
            target_speed -= step[a];
        }

        if (gamepad1.rightBumperWasPressed()) {
            a += 1;
        } else if (gamepad1.leftBumperWasPressed()) {
            a -= 1;
        }

        if (gamepad1.squareWasPressed()){
            testing_hoodT += step[a];
        } else if (gamepad1.triangleWasPressed()) {
            testing_hoodT -= step[a];
        }

        if (gamepad1.circleWasPressed()){
            testing_hoodL += step[a];
        } else if (gamepad1.crossWasPressed()) {
            testing_hoodL -= step[a];
        }

        if (gamepad1.dpadUpWasPressed()){
            testing_hoodR += step[a];
        } else if (gamepad1.dpadDownWasPressed()) {
            testing_hoodR -= step[a];
        }

        /* if (gamepad1.squareWasPressed()) {
            Kp += mode * step[a];
        } else if (gamepad1.triangleWasPressed()) {
            Ki += mode * step[a];
        } else if (gamepad1.circleWasPressed()) {
            Kd += mode * step[a];
        } else if (gamepad1.crossWasPressed()) {
            Kf += mode * step[a];
        }
         */

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        system_init.shooter.setVelocity(target_speed);

        system_init.HoodTop.setPosition(testing_hoodT);
        system_init.HoodLeft.setPosition(testing_hoodL);
        system_init.HoodRight.setPosition(testing_hoodR);

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

    //TODO: Get an approximation of an equation for the shooter velocity and hood angle needed to score from each distance
    public double[] StatCalc(){
        double[] placeholder = {1,2};
        return placeholder;
    }
}


