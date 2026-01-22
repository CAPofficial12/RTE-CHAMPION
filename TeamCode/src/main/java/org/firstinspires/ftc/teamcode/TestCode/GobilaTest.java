package org.firstinspires.ftc.teamcode.TestCode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Gobilda Test", group = "Sensor Test")
public class GobilaTest extends OpMode {

    GoBildaPinpointDriver pinpointDriver;

    @Override
    public void init(){
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        pinpointDriver.setOffsets(0,0,MM);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.recalibrateIMU();
        pinpointDriver.resetPosAndIMU();
        pinpointDriver.setPosition(new Pose2D(DistanceUnit.MM, 0,0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop(){
        telemetry.addData("X", pinpointDriver.getPosX(MM));
        telemetry.addData("Encoder X", pinpointDriver.getEncoderX());
        telemetry.addData("Encoder Y", pinpointDriver.getEncoderY());
        telemetry.addData("Y", pinpointDriver.getPosY(MM));
        telemetry.addData("Position", pinpointDriver.getPosition());
        telemetry.addData("Heading", pinpointDriver.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Status", pinpointDriver.getDeviceStatus());
        pinpointDriver.update();
        telemetry.update();
    }
}
