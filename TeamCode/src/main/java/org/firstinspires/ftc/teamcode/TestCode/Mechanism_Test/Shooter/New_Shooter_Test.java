package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

import java.util.concurrent.TimeoutException;

@TeleOp(name = "New shooter tuning")
public class New_Shooter_Test extends OpMode {
    System_init system = new System_init();
    TelemetryManager telemetryM;

    double HtargetV = 2000;
    double LtargetV = 1000;
    double curtarget = Shooter_Constants.target_speed;
    double F = Shooter_Constants.Kf;
    double P = Shooter_Constants.Kp;
    double D = Shooter_Constants.Kd;


    @Override
    public void init(){
        system.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        system.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.addLine("Init complete");
    }

    @Override
    public void loop(){

        if (gamepad1.leftBumperWasPressed()){
            curtarget = HtargetV;
        } else if (gamepad1.rightBumperWasPressed()) {
            curtarget = LtargetV;
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        system.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        system.shooter.setVelocity(curtarget);

        telemetryM.addData("Current Velocity", system.shooter.getVelocity());
        telemetryM.addData("Target Velocity", curtarget);
        telemetryM.addData("Error", curtarget - system.shooter.getVelocity());
        telemetryM.addData("P", P);
        telemetryM.addData("D", D);
        telemetryM.addData("F", F);
        telemetryM.update(telemetry);

    }
}
