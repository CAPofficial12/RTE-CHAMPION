
package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Running_Code.Auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Localisation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

//TODO: CHECK PATH OF ROBOT TO SEE IF IT MATCHES WITH WHAT WAS PLANNED
@Autonomous(name = "15 Artifact Auto Movement")
@Configurable // Panels
public class PedroAutonomous15Artifact extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    Localisation position = new Localisation();
    Pose init_position;
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        init_position = new Pose(position.merge().getX(DistanceUnit.INCH), position.merge().getY(DistanceUnit.INCH), position.merge().getHeading(AngleUnit.DEGREES));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(init_position);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing

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
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(67.831, 851985),

                                    new Pose(67.571, 117.423)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(156))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.571, 117.423),
                                    new Pose(55.064, 84.072),
                                    new Pose(17.023, 84.246)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.023, 84.246),

                                    new Pose(49.778, 94.324)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.778, 94.324),
                                    new Pose(47.158, 62.119),
                                    new Pose(17.370, 61.838)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.370, 61.838),

                                    new Pose(49.679, 92.931)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.679, 92.931),
                                    new Pose(59.662, 35.954),
                                    new Pose(9.554, 35.609)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.554, 35.609),

                                    new Pose(59.651, 83.866)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.651, 83.866),

                                    new Pose(128.540, 83.551)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.540, 83.551),

                                    new Pose(108.191, 107.885)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(150))

                    .build();
        }
    }
}
    