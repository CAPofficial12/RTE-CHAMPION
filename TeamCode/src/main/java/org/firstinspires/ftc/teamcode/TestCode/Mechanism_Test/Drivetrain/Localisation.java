package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;
import androidx.annotation.NonNull;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

import java.util.Timer;

@TeleOp(name = "Localisation go brr")
public class Localisation extends OpMode {
    System_init system = new System_init();
    TelemetryManager telemetryManager;
    Pose3D position;
    ElapsedTime clock_speed = new ElapsedTime();

    @Override
    public void init(){
        system.init(hardwareMap);
    }

    @Override
    public void start(){
        system.limelight3A.start();
    }

    @Override
    public void loop(){
        system.limelight3A.updateRobotOrientation(system.pinpoint.getHeading(AngleUnit.DEGREES));
        LLResult result = system.limelight3A.getLatestResult();
        //TODO: CHECK IF PANELS WORK

        telemetry.addData("Limelight result", system.limelight3A.getLatestResult());
        telemetry.addData("Limelight Ta", result.getTa());
        telemetry.addData("Megatag 2", result.getBotpose_MT2());
        telemetry.addData("MegaTag1", result.getBotpose());
        telemetry.addData("Distance", result.getBotposeAvgDist());
        telemetry.addData("Clock Speed", 1/clock_speed.seconds());
        telemetry.addData("Clock Speed", 1/clock_speed.seconds());
        telemetry.update();
        system.pinpoint.update();
    }

    //TODO: CHECK WHAT IMU ORIENTATION IS EXPECTED AT THE START
    public Pose3D getCamPosition(){
        system.limelight3A.updateRobotOrientation(system.pinpoint.getHeading(AngleUnit.RADIANS));
        LLResult result = system.limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            position = result.getBotpose_MT2();
            return position;
        } else {
            return null;
        }
    }

    public Pose2D conversion(@NonNull Pose3D position){
        double x = position.getPosition().x * 39.3701 + 72;
        double y = position.getPosition().y * 39.3701 + 72;
        double theta = position.getOrientation().getYaw(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x,y, AngleUnit.RADIANS, theta);
    }

    public Pose conversionPose(Pose2D position){
        double x = position.getX(DistanceUnit.INCH);
        double y = position.getY(DistanceUnit.INCH);
        double theta = position.getHeading(AngleUnit.RADIANS);
        return new Pose(x,y,theta);
    }

    public Pose conversion3D(Pose3D position){
        double x = position.getPosition().x * 39.3701 + 72;
        double y = position.getPosition().y * 39.3701 + 72;
        double theta = position.getOrientation().getYaw(AngleUnit.RADIANS);
        return new Pose(x,y,theta);
    }

    public Pose2D merge(){

        if (getCamPosition() == null){
            return system.pinpoint.getPosition();
        } else {
            system.pinpoint.setPosition(conversion(getCamPosition()));
            return conversion(getCamPosition());
        }
    }

    public double target_angle(){
        double delta_x = merge().getX(DistanceUnit.INCH) - 0;
        double delta_y = -merge().getY(DistanceUnit.INCH) + 144;
        return Math.PI - Math.atan2(delta_y, delta_x);

    }

}
