package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp (name = "Robot go brr")
public class Drive_brr extends OpMode {
    System_init system = new System_init();
    Drivetrain_Caertesian Drivetrain = new Drivetrain_Caertesian();
    double[] powers;
    double distance;

    @Override
    public void init(){
        system.init(hardwareMap);
    }

    @Override
    public void loop(){

        powers = Drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        distance = system.Brake.getDistance(DistanceUnit.MM);

        if (gamepad1.dpad_up && distance > 90 || gamepad1.right_trigger > 0.05) {
            system.intake.setPower(1);
            system.transfer.setPower(1);
        } else if (gamepad1.dpad_up && distance < 90) {
            system.intake.setPower(1);
            system.transfer.setPower(0);

        } else {
            system.transfer.setPower(0);
            system.intake.setPower(0);
        }

        if (gamepad1.right_bumper){
            system.shooter.setVelocity(2000);
        } else {
            system.shooter.setVelocity(0);
        }

        // Set drivetrain Powers
        system.frontLeftDrive.setPower(powers[0]);
        system.frontRightDrive.setPower(powers[1]);
        system.backLeftDrive.setPower(powers[2]);
        system.backRightDrive.setPower(powers[3]);

    }
}
