package org.firstinspires.ftc.teamcode.Mechanism;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants_TeleOp {

    public static double Kp_drive = 3.0;
    public static double Ki_drive = 0.0;
    public static double Kd_drive = 0.0;
    // PID For Turning
    public static double Kp_turn = 4.0;
    public static double Ki_turn = 0.0;
    public static double Kd_turn = 1.0;

    // PID For Shooter Front
    public static double Kp_Shoot = 4;
    public static double Ki_Shoot = 0.0;
    public static double Kd_Shoot = 1.7;
    public static double Kf_Shoot = 1; //Adjust after testing
    public static double Tolerance = 20;

    public static double maxAccel = 0.6;
    public static double maxPower = 1;
    public static  double Sensitivity = 1.5;
    public static  double SHOOTER = 1700;
    public static double Bound_Angle = 1;
    public static double Bound_Distance = 0.005;
    public static double radius = 2.45;
    public static double scaler = 1;
}
