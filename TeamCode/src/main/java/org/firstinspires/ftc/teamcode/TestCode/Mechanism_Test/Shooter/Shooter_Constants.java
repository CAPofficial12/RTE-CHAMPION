package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class Shooter_Constants {
    public static double Kp = 80;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 18.5;
    public static int target_speed = 1500;
    public static int target_hood = 0;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
    public static double angle = 0;
}
