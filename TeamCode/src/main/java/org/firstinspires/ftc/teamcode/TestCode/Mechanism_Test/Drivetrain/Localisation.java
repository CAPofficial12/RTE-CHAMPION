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

@TeleOp(name = "Localisation go brr")
public class Localisation extends OpMode {

    Limelight3A limelight3A;
    GoBildaPinpointDriver imu;
    Pose3D position;
    Drivetrain_Caertesian drivetrainCaertesian = new Drivetrain_Caertesian();

    @Override
    public void init(){

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

        imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        imu.setOffsets(0,0, DistanceUnit.MM);
        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        imu.recalibrateIMU();
        imu.resetPosAndIMU();
        imu.setHeading(90, AngleUnit.RADIANS);
        imu.setPosition(conversion(getCamPosition()));

    }


    @Override
    public void loop(){
        drivetrainCaertesian.loop();
        telemetry.addData("Gobilda position", imu.getPosition());
        telemetry.addData("Limelight Pos", getCamPosition());
        telemetry.update();
        imu.update();
    }

    public Pose3D getCamPosition(){
        limelight3A.updateRobotOrientation(imu.getHeading(AngleUnit.RADIANS));
        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            position = result.getBotpose_MT2();

        }
        return position;
    }

    public Pose2D conversion(@NonNull Pose3D position){
        double x = position.getPosition().x;
        double y = position.getPosition().y;
        double theta = position.getOrientation().getYaw(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x,y, AngleUnit.RADIANS, theta);
    }
}
