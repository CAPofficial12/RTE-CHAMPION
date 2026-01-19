package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Limelight", group = "Test")
public class Limelight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private double distance;

    @Override
    public void init(){
        limelight  = hardwareMap.get(Limelight3A.class, "Light");
        limelight.pipelineSwitch(8);
         imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientaion = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientaion.getYaw(AngleUnit.RADIANS));
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            distance = distanc(llResult.getTa());
            telemetry.addData("TX", llResult.getTx());
            telemetry.addData("TY", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("botPose", botPose.toString());
            telemetry.addData("Tad Distance", distance);
            telemetry.update();
        }

    }

    public double distanc(double Ta){
        double scale = 30665.95; // Retest Scale
        return scale/Ta;
    }
}
