package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Panels_Test;

import android.os.SystemClock;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

public class Panels_test extends OpMode {

    System_init system_init = new System_init();
    TelemetryManager telemetry;

    @Override
    public void init(){
        system_init.init(hardwareMap);
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop(){

        if (gamepad1.dpad_down){
            system_init.shooter.setPower(1);
        } else{
            system_init.shooter.setPower(0);
        }


        telemetry.addData("Position", system_init.shooter.getVelocity());
        telemetry.update();
    }
}
