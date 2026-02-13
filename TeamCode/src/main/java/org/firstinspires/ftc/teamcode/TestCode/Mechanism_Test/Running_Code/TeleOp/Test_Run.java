package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drive;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Transfer.Intake_Gate_Test;

@TeleOp (name = "Robot go brr")
public class Test_Run extends OpMode {
    System_init system = new System_init();
    Drive Drivetrain = new Drive();
    Intake_Gate_Test Transfer = new Intake_Gate_Test();
    Shooter shooter = new Shooter();
    Localisation localisation = new Localisation();
    double distance;

    @Override
    public void init(){
        system.init(hardwareMap);
        telemetry.addLine("Init complete");
    }

    @Override
    public void start(){
        system.limelight3A.start();
    }

    @Override
    public void loop(){
        Drivetrain.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, system.pinpoint.getHeading(AngleUnit.RADIANS));

        if (gamepad1.right_bumper){
            system.intake.setVelocity(1);
            system.transfer.setPower(1);
            Transfer.gateClose();
        }

        distance = Math.hypot((0-localisation.merge().getX(DistanceUnit.INCH)), (144-localisation.merge().getY(DistanceUnit.INCH)));
        system.shooter.setVelocity(shooter.statCalc(distance)[0]);
        system.Hood.setPosition(shooter.statCalc(distance)[1]);

    }
}
