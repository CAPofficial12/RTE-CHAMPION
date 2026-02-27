package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.Auto; // make sure this aligns with class location

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter_Constants;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Final Auto BLUE")
public class Final_Auto_BLUE extends OpMode {
    int target = Shooter_Constants.target_speed;

    System_init system = new System_init();
    TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    Localisation local = new Localisation();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    int counter = 0;

    Pose start = new Pose(20, 123, 135);
    Pose shooting = new Pose(59, 90, 135);
    Pose Intake1 = new Pose(13, 83);
    Pose Intake2 = new Pose(12, 60);
    Pose Intake3 = new Pose(7, 35);
    Pose Rest = new Pose(23,73, 20);

    public PathChain Score1;
    public PathChain CollectMid;
    public PathChain Scoremid;
    public PathChain Collectclose;
    public PathChain Shootclose;
    public PathChain CollectFar;
    public PathChain Shootfar;
    public PathChain Stop;


    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        system.init(hardwareMap);
        system.limelight3A.start();

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose( ));


        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths();
        follower.setStartingPose(new Pose(22, 123 , 135));

    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        follower.activateAllPIDFs();

        system.shooter.setVelocity(target);
        system.intake.setPower(1);

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    private Path scorePreload;

    public void buildPaths() {
        Score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                start,
                                shooting
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        CollectMid = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shooting,
                                new Pose(43, 59.000),
                                Intake2
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Scoremid = follower.pathBuilder().addPath(
                        new BezierCurve(
                                Intake2,
                                new Pose(39, 58),
                                shooting
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))

                .build();

        Collectclose = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shooting,
                                new Pose(54, 70),
                                Intake1
                        )
                ).setTangentHeadingInterpolation()
                .setGlobalDeceleration(1.25)

                .build();

        Shootclose = follower.pathBuilder().addPath(
                        new BezierLine(
                                Intake1,
                                shooting
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))

                .build();

        CollectFar = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shooting,
                                new Pose(49.000, 32.000),
                                Intake3
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Shootfar = follower.pathBuilder().addPath(
                        new BezierLine(
                                Intake3,
                                shooting
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))

                .build();

        Stop = follower.pathBuilder().addPath(
                        new BezierLine(
                                shooting,
                                Rest
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .setTimeoutConstraint(0)

                .build();


    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()){
                    follower.followPath(Score1);
                    setPathState(8);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(CollectMid);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Scoremid);
                    setPathState(8);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Collectclose);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Shootclose);
                    setPathState(8);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(CollectFar);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Shootfar);
                    setPathState(8);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Stop);
                    setPathState(-1);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    system.transfer.setPower(1);
                    follower.wait(1000);
                    system.transfer.setPower(0);
                    follower.wait(2000);
                    system.transfer.setPower(1);
                    counter += 2;
                    setPathState(counter);
                }




        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
