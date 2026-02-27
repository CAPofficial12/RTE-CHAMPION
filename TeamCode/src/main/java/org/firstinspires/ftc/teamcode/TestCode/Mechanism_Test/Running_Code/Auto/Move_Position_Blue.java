package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.Auto;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter_Constants;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Move Autonomous nRED")
public class Move_Position_Blue extends OpMode {
    System_init system = new System_init();
    Drivetrain_Caertesian drive = new Drivetrain_Caertesian();
    Localisation local = new Localisation();
    TelemetryManager telemetryM;
    double angle = Shooter_Constants.angle;
    private boolean forward = true;
    static PoseHistory poseHistory;
    private Path forwards;

    double distance;
    boolean intake;
    double target = Shooter_Constants.target_speed;
    PIDFCoefficients pidf = Shooter_Constants.pidfCoefficients;

    @Override
    public void init() {
        system.init(hardwareMap);

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.setStartingPose(new Pose(22,123));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {

        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();

        forwards = new Path(new BezierLine(follower.getPose(), new Pose(59,90)));
        forwards.setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(110));
        forwards.setTimeoutConstraint(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {

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
            double [] powers = drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            system.frontLeftDrive.setPower(powers[0]);
            system.frontRightDrive.setPower(powers[1]);
            system.backLeftDrive.setPower(powers[2]);
            system.backRightDrive.setPower(powers[3]);
        }

        if (gamepad1.leftBumperWasPressed()){
            intake = !intake;
        }
        distance = system.Brake.getDistance(DistanceUnit.MM);
        if (intake && distance > 90) {
            system.intake.setPower(1);
            system.transfer.setPower(0.7);
        } else if (intake && distance < 90) {
            system.intake.setPower(1);
            system.transfer.setPower(0);
        } else if (gamepad1.right_bumper) {
            system.intake.setPower(1);
            system.transfer.setPower(0.7);
        } else {
            system.transfer.setPower(0);
            system.intake.setPower(0);
        }

        system.shooter.setVelocity(1200);

        telemetryM.addData("Target", target);
        telemetryM.addData("Shootr speed", system.shooter.getVelocity());
        telemetryM.debug("Position", follower.getPose());
        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}