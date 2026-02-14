package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;
import androidx.annotation.NonNull;

import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    Drive drive = new Drive();
    TelemetryManager telemetryManager;
    FieldManager fieldManager;
    Pose3D position;
    ElapsedTime clock_speed = new ElapsedTime();
    boolean Robot;

    @Override
    public void init(){
        system.init(hardwareMap);
        fieldManager.init();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        system.limelight3A.start();
        system.pinpoint.setHeading(0, AngleUnit.RADIANS);
    }

    @Override
    public void start(){
        system.pinpoint.setPosition(conversion(getCamPosition()));
    }

    @Override
    public void loop(){
        drive.drivetrain();
        merge();

        //TODO: CHECK IF PANELS WORK
        fieldManager.clearFill();
        fieldManager.moveCursor(merge().getX(DistanceUnit.INCH), merge().getY(DistanceUnit.INCH));
        fieldManager.circle(1);

        telemetryManager.debug("Gobilda position", system.pinpoint.getPosition());
        telemetryManager.debug("Limelight Pos", getCamPosition());
        telemetryManager.debug("Current Position", merge());
        telemetryManager.debug("Clock Speed", 1/clock_speed.seconds());

        telemetry.addData("Gobilda position", system.pinpoint.getPosition());
        telemetry.addData("Limelight Pos", getCamPosition());
        telemetry.addData("Current Position", merge());
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
        double x = position.getPosition().x;
        double y = position.getPosition().y;
        double theta = position.getOrientation().getYaw(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x,y, AngleUnit.RADIANS, theta);
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
