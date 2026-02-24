package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import java.util.Objects;

@Autonomous(name = "Pedro Pathing Full Autonomous")
@Configurable // Panels
public class PedroAutonomousFull extends OpMode {
    System_init system_init = new System_init();

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    boolean shoot = false;

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
        //Set motor power
        system_init.shooter.setPower(1);
        system_init.intake.setPower(1);

        follower.update(); // Update Pedro Pathing
        if (Objects.equals(follower.getPose(), new Pose(45, 98, 129))|| system_init.Brake.getDistance(DistanceUnit.MM) < 90){
            system_init.transfer.setPower(1);
        }
        autonomousPathUpdate(); // Update autonomous state machine



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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.000, 135.000),

                                    new Pose(45.000, 98.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(129))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(45.000, 98.000),
                                    new Pose(43.000, 79.000),
                                    new Pose(17.000, 85.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 85.000),

                                    new Pose(45.000, 98.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))

                    .build();
        }
    }


    public void autonomousPathUpdate() {

        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
