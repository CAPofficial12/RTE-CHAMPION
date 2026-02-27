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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave Autonomous")
public class Leave_AUTO extends OpMode {
    System_init system_init = new System_init();

    TelemetryManager telemetryM;

    public static double DISTANCE = 40;
    private boolean forward = true;
    static PoseHistory poseHistory;
    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        system_init.init(hardwareMap);

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.setStartingPose(new Pose(72, 72));
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

        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);

        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            forward = true;
            follower.followPath(forwards);
        }

        if (follower.atParametricEnd()){
            stopRobot();
        }

        telemetryM.debug("Position", follower.getPose());
        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}
