package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Localisation {
    Limelight3A limelight3A;
    IMU imu;
    double X,Y,Position;

    public Pose getCamPosition(){
        YawPitchRollAngles orientatiuon = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientatiuon.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                X = result.getBotpose().getPosition().x;
                Y = result.getBotpose().getPosition().y;
            }
        }
        return new Pose(X,Y, imu.getRobotYawPitchRollAngles().getYaw());
    }
}
