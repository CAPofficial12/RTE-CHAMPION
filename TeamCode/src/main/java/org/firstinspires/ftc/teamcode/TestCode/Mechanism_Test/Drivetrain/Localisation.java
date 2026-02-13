package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Localisation go brr")
public class Localisation extends OpMode {
    System_init system = new System_init();
    Drive drive = new Drive();
    Pose3D position;
    boolean Robot;

    @Override
    public void init(){
        system.init(hardwareMap);
        system.limelight3A.start();
        system.pinpoint.setHeading(0, AngleUnit.RADIANS);
        system.pinpoint.setPosition(conversion(getCamPosition()));
    }


    @Override
    public void loop(){
        drive.drivetrain();
        merge();

        telemetry.addData("Gobilda position", system.pinpoint.getPosition());
        telemetry.addData("Limelight Pos", getCamPosition());
        telemetry.addData("Current Position", merge());
        telemetry.update();
        system.pinpoint.update();
    }

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
        return Math.atan2(delta_y, delta_x);
    }
}
