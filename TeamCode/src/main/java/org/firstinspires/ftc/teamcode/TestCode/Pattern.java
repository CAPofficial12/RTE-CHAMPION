package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Obelisk Checker")
public class Pattern extends OpMode {
    Limelight3A limelight3A;
    GoBildaPinpointDriver imu;
    int obelisk;

    @Override
    public void init(){

        limelight3A  = hardwareMap.get(Limelight3A.class, "Light");
        limelight3A.pipelineSwitch(7);
        limelight3A.start();

        imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult.isValid()) {
            obelisk = llResult.getFiducialResults().get(0).getFiducialId();
            if (obelisk == 21) {
                telemetry.addLine("GPP");
            } else if (obelisk == 22) {
                telemetry.addLine("GPG");
            } else if (obelisk == 23) {
                telemetry.addLine("GGP");
            }
        }
        else {
            telemetry.addLine("MONEY");
        }

    }
}
