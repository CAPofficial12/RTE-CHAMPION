package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

public class Shooter{

    System_init system_init = new System_init();

    public void shooter_speed(){  //TODO: add all shoooter commands here
        double target_speed = statCalc(50)[0];
        double target_hood = statCalc(50)[1];
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0,0,0,0);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        system_init.shooter.setVelocity(target_speed);
        system_init.Hood.setPosition(target_hood);
    }

    public double[] statCalc(double distance){ //TODO: Get an approximation of an equation for the shooter velocity and hood angle needed to score from each distance
        double[] placeholder = {1,2};
        return placeholder;
    }
}