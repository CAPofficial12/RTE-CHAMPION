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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain.Drivetrain_Caertesian;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Temp Autonomous")
public class Temo_Auto extends OpMode {
    System_init system = new System_init();
    Drivetrain_Caertesian drive = new Drivetrain_Caertesian();

    TelemetryManager telemetryM;
    static PoseHistory poseHistory;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // All positions
    Pose start = new Pose(123, 124, 45);
    Pose shoot = new Pose(85, 83, 45);

    private Path Score_1;

    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

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
        follower.setStartingPose(start);
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

        Score_1 = new Path(new BezierLine(start, shoot));
        Score_1.setConstantHeadingInterpolation(45);
        Score_1.setTranslationalConstraint(1);
        Score_1.setHeadingConstraint(45);

        follower.followPath(Score_1);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();

        system.shooter.setPower(1);

        if (!follower.isBusy()) {
            follower.followPath(Score_1);
        }

        if(follower.atParametricEnd()){
            stopRobot();
            system.intake.setPower(1);
            system.transfer.setPower(0.5);
        }

        telemetryM.debug("Position", follower.getPose());
        telemetryM.update(telemetry);
    }
}