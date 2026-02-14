
package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;
import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

// TODO: CHECK IF WORKS: IF WORKS, SET THIS CODE AS MAIN DRIVETRAIN
@TeleOp(name = "Drivetrain go brr")
public class Drive extends OpMode {
    // This declares the four motors needed
    System_init system = new System_init();
    Localisation localisation = new Localisation();
    TelemetryManager panels;
    double rotation;
    boolean Robot = true;


    @Override
    public void init() {
        system.init(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        system.pinpoint.setHeading(0, AngleUnit.RADIANS);
        panels.debug("Init complete");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            system.pinpoint.recalibrateIMU();
        }

        drivetrain();

        if (gamepad1.right_trigger > 0){
            if (Math.abs(localisation.target_angle() - system.pinpoint.getHeading(AngleUnit.RADIANS)) > 1){
                rotation = Math.signum(localisation.target_angle()) * -1;
            }
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotation, system.pinpoint.getHeading(AngleUnit.RADIANS));
        }

        panels.debug("Robot Relative Drive", Robot);
        panels.update(telemetry);
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate, double heading) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - heading);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        system.frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        system.frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        system.backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        system.backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void drivetrain(){
        if (Robot) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, system.pinpoint.getHeading(AngleUnit.RADIANS));
        }

        if (gamepad1.leftBumperWasPressed()){
            Robot = true;
        } else if (gamepad1.rightBumperWasPressed()) {
            Robot = false;
        }
    }
}
