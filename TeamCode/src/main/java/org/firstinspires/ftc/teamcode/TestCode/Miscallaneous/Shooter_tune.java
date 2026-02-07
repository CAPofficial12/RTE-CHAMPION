package org.firstinspires.ftc.teamcode.TestCode.Miscallaneous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.System_init;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Shooter Tuning")
public class Shooter_tune extends OpMode {
    System_init system_init = new System_init();

    double[] step = {100,10,1,0.1,0.01,0.001};
    Map<Double, Double> speed_distance = new HashMap<>();

    int a = 0;
    int mode = 1;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    double[] target_speeds = {1000,2000};
    double target_speed = target_speeds[1];
    double testing_speed = 0;
    double testing_distance = 0;
    double testing_hood = 0;
    boolean type = false;

    @Override
    public void init() {
        system_init.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.rightStickButtonWasReleased()){
            type = !type;
        }

        if (type) {
            if (gamepad1.left_bumper) {
                target_speed = target_speeds[1];
            } else if (gamepad1.right_bumper) {
                target_speed = target_speeds[0];
            }

            if (gamepad1.dpadUpWasPressed()) {
                a += 1;
            } else if (gamepad1.dpadDownWasPressed()) {
                a -= 1;
            }

            if (gamepad1.dpad_right) {
                mode = 1;
            } else if (gamepad1.dpad_left) {
                mode = -1;
            }

            if (gamepad1.y) {
                Kp += mode * step[a];
            } else if (gamepad1.b) {
                Ki += mode * step[a];
            } else if (gamepad1.a) {
                Kd += mode * step[a];
            } else if (gamepad1.x) {
                Kf += mode * step[a];
            }

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
            system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            system_init.shooter.setVelocity(target_speed);

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
        } else{
            system_init.shooter.setVelocity(testing_speed);

            if (gamepad1.dpadRightWasPressed()) {
                a += 1;
            } else if (gamepad1.dpadLeftWasPressed()) {
                a -= 1;
            }

            if (gamepad1.dpadUpWasPressed()) {
                testing_speed += step[a];
            } else if (gamepad1.dpadLeftWasPressed()) {
                testing_speed -= step[a];
            }

            if (gamepad1.aWasPressed()){
                testing_distance += step[a];
            } else if (gamepad1.backWasPressed()) {
                testing_distance -= step[a];
            }

            if (gamepad1.start){
                telemetry.addLine("Speed Was Saved");
            } else if (gamepad1.startWasPressed()) {
                speed_distance.put(testing_speed, testing_distance);
            }


            telemetry.addData("Current Velocity", system_init.shooter.getVelocity());
            telemetry.addData("Current velocity in degrees", system_init.shooter.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Target Velocity", testing_speed);
            telemetry.addData("Step", step[a]);
            telemetry.update();

        }
    }
}


