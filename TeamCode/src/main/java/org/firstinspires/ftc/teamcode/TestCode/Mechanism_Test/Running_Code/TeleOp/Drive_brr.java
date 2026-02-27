package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.TeleOp;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;

import androidx.lifecycle.DefaultLifecycleObserver;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter_Constants;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp (name = "Robot go brr")
public class Drive_brr extends OpMode {
    public static Follower follower;
    System_init system = new System_init();
    Drivetrain_Caertesian Drivetrain = new Drivetrain_Caertesian();
    TelemetryManager telemetryM;
    double[] powers;
    double distance;
    LLResult result;
    boolean intake;
    double target = Shooter_Constants.target_speed;
    PIDFCoefficients pidf = Shooter_Constants.pidfCoefficients;
    Localisation local = new Localisation();
    static PoseHistory poseHistory;
    private Path forwards;

    @Override
    public void init(){
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        system.init(hardwareMap);

        system.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);
    }

    @Override
    public void start(){
        system.limelight3A.start();

        follower.activateAllPIDFs();

        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(103, 103)));
        forwards.setLinearHeadingInterpolation(0, 30);
        forwards.setTranslationalConstraint(1);
        forwards.setHeadingConstraint(45);
        forwards.setTimeoutConstraint(50);
        follower.followPath(forwards);
    }

    @Override
    public void loop(){

        if (gamepad1.circle){
            if (!follower.isBusy()){
                follower.followPath(forwards);
            }
            if (follower.atParametricEnd()){
                stopRobot();
            }
            follower.update();

        } else if (gamepad1.circleWasReleased()) {
            follower.breakFollowing();
        }
        else {
            double [] powers = Drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            system.frontLeftDrive.setPower(powers[0]);
            system.frontRightDrive.setPower(powers[1]);
            system.backLeftDrive.setPower(powers[2]);
            system.backRightDrive.setPower(powers[3]);
        }

        distance = system.Brake.getDistance(DistanceUnit.MM);
        if (gamepad1.leftBumperWasPressed()){
            intake = !intake;
        }

        if (intake && distance > 90 || gamepad1.right_bumper) {
            system.intake.setPower(1);
            system.transfer.setPower(0.7);
        } else if (intake && distance < 90) {
            system.intake.setPower(1);
            system.transfer.setPower(0);
        } else {
            system.transfer.setPower(0);
            system.intake.setPower(0);
        }


        system.shooter.setVelocity(target);
        telemetry.addData("Shootr speed", target);
        telemetry.update();

    }
}
