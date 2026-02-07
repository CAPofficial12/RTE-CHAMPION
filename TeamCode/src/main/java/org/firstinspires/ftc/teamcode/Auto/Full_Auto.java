package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Mechanism.System_init;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Transfer.Intake_Gate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous 3 Segment", group = "Autonomous")
public class Full_Auto extends OpMode {

    System_init system_init = new System_init();
    Shooter shooter = new Shooter();
    Intake_Gate intakeGate = new Intake_Gate(); // TODO: Seperate Intake and Gate into seperate files
    Localisation localisation = new Localisation();
    Timer pathTimer;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private int Pathnum = 2;
    private Pose startPose; // Start Pose of our robot.
    private final Pose SmallscorePose = new Pose(56, 8, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BigscorePose = new Pose(45, 98, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(13, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseReverse = new Pose(34, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2Pose = new Pose(15, 64, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseReverse = new Pose(34, 64, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3Pose = new Pose(13, 86, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseReverse = new Pose(34, 86, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose Goal = new Pose(5,144);
    double distance;
    @Override
    public void init() {
        pathTimer = new Timer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        system_init.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        Paths(follower);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        startPose = localisation.getCamPosition();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        try {
            pathState = autonomousPathUpdate(); // Update autonomous state machine
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        distance = follower.getPose().distanceFrom(Goal);
        intakeGate.intake();
        shooter.SpeedCalc(distance);


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    PathChain preload,Path1, Path2, Path3, Path4;
    public void Paths(Follower follower) {

        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                SmallscorePose
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), 110)
                .build();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(36.000, 41.000),
                                pickup1Pose
                        )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .addPath(
                            new BezierLine (
                                    pickup1Pose,
                                    SmallscorePose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                SmallscorePose,
                                new Pose(63.000, 65.000),
                                pickup2Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                .addPath(
                        new BezierCurve(
                                pickup2Pose,
                                new Pose(61.000, 74.000),
                                BigscorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    BigscorePose,
                                    new Pose(17.000, 103.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))

                    .addPath(
                            new BezierCurve(
                                    new Pose(17.000, 103.000),
                                    new Pose(16.000, 79.000),
                                    pickup3PoseReverse
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(45))

                    .addPath(
                            new BezierLine(
                                    pickup3PoseReverse,
                                    BigscorePose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    BigscorePose,
                                    new Pose(106.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();
        }


    public int autonomousPathUpdate() throws InterruptedException {
        switch (pathState){
            case 0:
                follower.followPath(preload);
                setPathState(1);
                break;
            case 1:
                intakeGate.gateOpen();
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeGate.gateClose();
                    setPathState(Pathnum);
                    Pathnum += 1;
                    break;
                }
            case 2:
                follower.followPath(Path1);
                setPathState(1);
                break;
            case 3:
                follower.followPath(Path2);
                setPathState(1);
                break;

            case 4:
                follower.followPath(Path3);
                setPathState(1);
                break;
            case 5:
                follower.followPath(Path4);
                break;

        }
        return pathState;
    }

    public void setPathState(int pState){
        pathTimer.resetTimer();
        pathState = pState;
    }
}