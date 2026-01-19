package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous 3 Segment", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous3Segment extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(36.000, 41.000),
                                    new Pose(13.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .addPath(
                            new BezierLine(
                                    new Pose(13.000, 36.000),
                                    new Pose(56.000, 8.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(63.000, 65.000),
                                    new Pose(15.000, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .addPath(
                            new BezierCurve(
                                    new Pose(15.000, 64.000),
                                    new Pose(61.000, 74.000),
                                    new Pose(45.000, 98.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45.000, 98.000),
                                    new Pose(17.000, 103.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))

                    .addPath(
                            new BezierCurve(
                                    new Pose(17.000, 103.000),
                                    new Pose(16.000, 79.000),
                                    new Pose(34.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(45))

                    .addPath(
                            new BezierLine(
                                    new Pose(34.000, 86.000),
                                    new Pose(45.000, 98.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45.000, 98.000),
                                    new Pose(106.000, 33.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}